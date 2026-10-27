/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   摇杆读取与云台手动控制
  ******************************************************************************
  */
#include "joystick.h"

#if (SERVO_DRV_PCA9685 != 0)
  #include "bsp_servo.h"     /* PCA9685（I2C2），舵机保持接在驱动板上 */
#else
  #include "bsp_pwm.h"       /* TIM2 直驱 PA0/PA1 */
#endif

#define JOY_PAN_CH    0U
#define JOY_TILT_CH   1U

/* 统一出口：两种驱动都是 500~2500us */
#if (SERVO_DRV_PCA9685 != 0)
  #define SERVO_WRITE(ch, us)  (void)BSP_Servo_SetPulseUs((ch), (us))
#else
  #define SERVO_WRITE(ch, us)  (void)BSP_PWM_SetPulseUs((ch), (us))
#endif

/* ---------------------------------------------------------------------------
 * DMA 缓冲区必须放在 AXI SRAM（0x2400_0000）：
 *   H7 的 DTCM(0x2000_0000) 不被 DMA1/DMA2 访问，放在这里读到的永远是 0。
 *   链接脚本里已定义 .axi_sram 段，MPU Region 0 也把它配成了 Non-Cacheable。
 * ------------------------------------------------------------------------- */
__attribute__((section(".axi_sram"), aligned(32)))
static volatile uint16_t s_adc_dma[2];

static uint8_t s_started = 0U;
static uint8_t s_sw_last = 1U;
static uint8_t s_joy_valid = 0U;           /* 上电后检测到摇杆才置 1 */
static uint16_t s_last_us[2] = {0U, 0U};   /* 上次写出的脉宽，避免重复写 */

/* 一阶低通状态 */
static float s_fx = 0.0f;
static float s_fy = 0.0f;

#define JOY_FILTER_ALPHA  0.30f

volatile int16_t  g_stick_x = 0;
volatile int16_t  g_stick_y = 0;
volatile uint16_t g_joy_raw[2] = {JOY_ADC_CENTER, JOY_ADC_CENTER};
volatile uint8_t  g_joy_key = 0U;

/* -------------------------------------------------------------------------- */
void Joystick_Init(void)
{
  s_fx = 0.0f;
  s_fy = 0.0f;
  g_stick_x = 0;
  g_stick_y = 0;
  g_joy_key = 0U;
  s_sw_last = 1U;
  s_last_us[0] = 0U;   /* 清缓存，保证第一帧一定写出 */
  s_last_us[1] = 0U;

  /* 连续转换 + 循环 DMA：转换结果一直刷新 s_adc_dma[] */
  s_started = (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_dma, 2U) == HAL_OK) ? 1U : 0U;
}

/* ADC 原始值 -> -100~100 */
static int16_t raw_to_stick(uint16_t raw)
{
  int32_t v;

  v = ((int32_t)raw - (int32_t)JOY_ADC_CENTER) * JOY_FULL_SCALE /
      (int32_t)((JOY_ADC_CENTER > 0) ? (4095 - JOY_ADC_CENTER) : 1);

  if (v > JOY_FULL_SCALE)  v = JOY_FULL_SCALE;
  if (v < -JOY_FULL_SCALE) v = -JOY_FULL_SCALE;

  return (int16_t)v;
}

void joystick_read(void)
{
  int16_t rx;
  int16_t ry;

  if (s_started == 0U)
  {
    return;   /* ADC/DMA 没起来，保持上次值 */
  }

  g_joy_raw[0] = s_adc_dma[0];
  g_joy_raw[1] = s_adc_dma[1];

  /* ---- 上电保护：检测不到摇杆就锁中位 -------------------------------
     模拟输入悬空时 H7 一般读到 100~300，如果直接当满行程用，
     一上电舵机就会打到 0°/180° 撞限位。这里要求两个通道都落在
     合理区间才解锁，之后全行程（含 0/4095）都可以正常使用。 */
  if (s_joy_valid == 0U)
  {
    if ((g_joy_raw[0] > JOY_RAW_MIN) && (g_joy_raw[0] < JOY_RAW_MAX) &&
        (g_joy_raw[1] > JOY_RAW_MIN) && (g_joy_raw[1] < JOY_RAW_MAX))
    {
      s_joy_valid = 1U;
    }
    else
    {
      s_fx = 0.0f;
      s_fy = 0.0f;
      g_stick_x = 0;
      g_stick_y = 0;
      return;     /* 摇杆未接：保持中位 */
    }
  }

  rx = raw_to_stick(g_joy_raw[0]);
  ry = raw_to_stick(g_joy_raw[1]);

#if (JOY_X_INVERT != 0)
  rx = (int16_t)(-rx);
#endif
#if (JOY_Y_INVERT != 0)
  ry = (int16_t)(-ry);
#endif

  /* 一阶低通：抑制 ADC 噪声与机械抖动 */
  s_fx += JOY_FILTER_ALPHA * ((float)rx - s_fx);
  s_fy += JOY_FILTER_ALPHA * ((float)ry - s_fy);

  /* 死区：滤波后仍在 ±JOY_DEADBAND 内则归零，防止中位高频抖动 */
  if (s_fx > -(float)JOY_DEADBAND && s_fx < (float)JOY_DEADBAND)
  {
    s_fx = 0.0f;
  }
  if (s_fy > -(float)JOY_DEADBAND && s_fy < (float)JOY_DEADBAND)
  {
    s_fy = 0.0f;
  }

  g_stick_x = (int16_t)((s_fx >= 0.0f) ? (s_fx + 0.5f) : (s_fx - 0.5f));
  g_stick_y = (int16_t)((s_fy >= 0.0f) ? (s_fy + 0.5f) : (s_fy - 0.5f));
}

/* -------------------------------------------------------------------------- */
void key_scan(void)
{
  uint8_t now;

  /* SW 内部上拉：未按下为高，按下为低 */
  now = (HAL_GPIO_ReadPin(JOY_SW_GPIO_Port, JOY_SW_Pin) == GPIO_PIN_RESET) ? 0U : 1U;

  g_joy_key = ((s_sw_last == 1U) && (now == 0U)) ? 1U : 0U;   /* 下降沿 */
  s_sw_last = now;
}

/* -------------------------------------------------------------------------- */
/* 角度 0~180° -> 脉宽 500~2500us（两种驱动共用） */
static uint16_t angle_to_pulse(float deg)
{
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  return (uint16_t)(500.0f + deg * (2000.0f / 180.0f) + 0.5f);
}

/* 值没变就不重复写（PCA9685 走 I2C，减少总线占用） */
static void servo_apply(uint8_t ch, uint16_t pulse_us)
{
  if (pulse_us != s_last_us[ch])
  {
    SERVO_WRITE(ch, pulse_us);
    s_last_us[ch] = pulse_us;
  }
}

void servo_control(void)
{
  float    pan_deg;
  float    tilt_deg;
  uint16_t us_pan;
  uint16_t us_tilt;

  /* 按下摇杆帽：立即回中 */
  if (g_joy_key != 0U)
  {
    g_joy_key = 0U;
    s_fx = 0.0f;
    s_fy = 0.0f;
    g_stick_x = 0;
    g_stick_y = 0;
    servo_apply(JOY_PAN_CH,  angle_to_pulse(90.0f));
    servo_apply(JOY_TILT_CH, angle_to_pulse(90.0f));
    return;
  }

  /* 控制量(-100~100) -> 角度(0~180, 90 为中位) */
  pan_deg  = 90.0f + (float)g_stick_x * 0.9f;
  tilt_deg = 90.0f + (float)g_stick_y * 0.9f;

  /* ----------------------------------------------------------------------
   * 【摄像头到货后的升级点】
   * 把视觉偏差赋给 g_stick_x / g_stick_y 即可复用整条链路：
   *
   *     g_stick_x = PID_Update(&pid_pan,  (float)(target_px - center_px));
   *     g_stick_y = PID_Update(&pid_tilt, (float)(target_py - center_py));
   *
   * 注意 PID 输出同样要限幅到 ±100，然后本函数照常换算角度。
   * 舵机接口、限位、脉宽换算都不用动。
   * -------------------------------------------------------------------- */
  us_pan  = angle_to_pulse(pan_deg);
  us_tilt = angle_to_pulse(tilt_deg);

  servo_apply(JOY_PAN_CH,  us_pan);
  servo_apply(JOY_TILT_CH, us_tilt);
}
