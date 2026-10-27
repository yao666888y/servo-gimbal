/**
  ******************************************************************************
  * @file    joystick.c
  * @brief   摇杆读取与云台手动控制
  ******************************************************************************
  */
#include "joystick.h"
#include "bsp_pwm.h"

/* ---------------------------------------------------------------------------
 * DMA 缓冲区必须放在 AXI SRAM（0x2400_0000）：
 *   H7 的 DTCM(0x2000_0000) 不被 DMA1/DMA2 访问，放在这里读到的永远是 0。
 *   链接脚本里已定义 .axi_sram 段，MPU Region 0 也把它配成了 Non-Cacheable。
 * ------------------------------------------------------------------------- */
__attribute__((section(".axi_sram"), aligned(32)))
static volatile uint16_t s_adc_dma[2];

static uint8_t s_started = 0U;
static uint8_t s_sw_last = 1U;

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
void servo_control(void)
{
  float pan_deg;
  float tilt_deg;

  /* 按下摇杆帽：立即回中 */
  if (g_joy_key != 0U)
  {
    g_joy_key = 0U;
    s_fx = 0.0f;
    s_fy = 0.0f;
    g_stick_x = 0;
    g_stick_y = 0;
    BSP_PWM_SetAngle(SERVO_PAN,  SERVO_ANGLE_MID_DEG);
    BSP_PWM_SetAngle(SERVO_TILT, SERVO_ANGLE_MID_DEG);
    return;
  }

  /* 控制量(-100~100) -> 角度(0~180, 90 为中位) */
  pan_deg  = SERVO_ANGLE_MID_DEG + (float)g_stick_x * (SERVO_ANGLE_MID_DEG / 100.0f);
  tilt_deg = SERVO_ANGLE_MID_DEG + (float)g_stick_y * (SERVO_ANGLE_MID_DEG / 100.0f);

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
  BSP_PWM_SetAngle(SERVO_PAN,  pan_deg);
  BSP_PWM_SetAngle(SERVO_TILT, tilt_deg);
}
