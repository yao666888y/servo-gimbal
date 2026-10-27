/**
  ******************************************************************************
  * @file    bsp_servo.c
  * @brief   PCA9685 16 路舵机驱动板驱动（I2C2, 100kHz）
  ******************************************************************************
  * 关键换算（12-bit @ 50.03Hz）：
  *   周期 = (PRESCALE+1) * 4096 / 25MHz = 122 * 4096 / 25e6 = 19.989 ms
  *   1 LSB = 4.88 us
  *   off_count = round(pulse_us * 4096 / 19989)
  *   500us -> 102, 1500us -> 307, 2500us -> 512
  ******************************************************************************
  */
#include "bsp_servo.h"

#define PCA9685_ADDR8        (PCA9685_I2C_ADDR_7B << 1)

#define PCA9685_REG_MODE1    0x00U
#define PCA9685_REG_MODE2    0x01U
#define PCA9685_REG_LED0     0x06U
#define PCA9685_REG_PRESCALE 0xFEU

#define PCA9685_MODE1_RESTART 0x80U
#define PCA9685_MODE2_OUTDRV  0x04U

#define PCA9685_I2C_TIMEOUT  20U

#define PCA9685_PERIOD_NS    19989U  /* us */
#define PCA9685_COUNTS       4096U

static uint16_t s_pulse_us[SERVO_CH_NUM] = {SERVO_PULSE_MID_US, SERVO_PULSE_MID_US};
static uint8_t  s_ready = 0U;

/* 上电自检回读结果（SWD 可直接读） */
volatile uint8_t g_servo_selftest[8] = {0};

/* -------------------------------------------------------------------------- */
static HAL_StatusTypeDef pca_write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDR8, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, PCA9685_I2C_TIMEOUT);
}

static HAL_StatusTypeDef pca_read(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c2, PCA9685_ADDR8, reg, I2C_MEMADD_SIZE_8BIT,
                          value, 1U, PCA9685_I2C_TIMEOUT);
}

/* 读任意寄存器（调试用：可直接看 MODE1/PRESCALE/LED0 的真实值） */
HAL_StatusTypeDef BSP_Servo_ReadReg(uint8_t reg, uint8_t *value)
{
  return pca_read(reg, value);
}

/* 脉宽 us -> 12bit 计数值 */
static uint16_t pulse_to_counts(uint16_t pulse_us)
{
  uint32_t counts = ((uint32_t)pulse_us * PCA9685_COUNTS + (PCA9685_PERIOD_NS / 2U))
                    / PCA9685_PERIOD_NS;
  if (counts > (PCA9685_COUNTS - 1U))
  {
    counts = PCA9685_COUNTS - 1U;
  }
  return (uint16_t)counts;
}

static uint16_t clamp_pulse(uint16_t pulse_us)
{
  if (pulse_us < SERVO_PULSE_MIN_US) return SERVO_PULSE_MIN_US;
  if (pulse_us > SERVO_PULSE_MAX_US) return SERVO_PULSE_MAX_US;
  return pulse_us;
}

/* -------------------------------------------------------------------------- */
HAL_StatusTypeDef BSP_Servo_Init(void)
{
  HAL_StatusTypeDef st;
  uint8_t mode1;

  s_ready = 0U;

  /* 1) 先关输出，避免上电瞬间舵机乱摆 */
  HAL_GPIO_WritePin(SERVO_OE_GPIO_Port, SERVO_OE_Pin, GPIO_PIN_SET);
  HAL_Delay(5U);

  /* 2) 器件在线检查 */
  st = HAL_I2C_IsDeviceReady(&hi2c2, PCA9685_ADDR8, 3U, PCA9685_I2C_TIMEOUT);
  if (st != HAL_OK) return st;

  /* 3) 进 SLEEP，写分频值。
        注意 MODE1 的 bit5 = AI（自动递增），复位默认是 0。
        必须置 1，否则 4 字节写 LEDn_ON/OFF 只会反复写同一个寄存器，
        结果 OFF 寄存器永远是 0 → 输出恒低 → 没有任何 PWM。 */
  st = pca_write(PCA9685_REG_MODE1, 0x31U);        /* SLEEP=1, AI=1, ALLCALL=1 */
  if (st != HAL_OK) return st;

  st = pca_write(PCA9685_REG_PRESCALE, 121U);      /* 50.03 Hz */
  if (st != HAL_OK) return st;

  /* 4) 推挽输出模式 */
  st = pca_write(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV);
  if (st != HAL_OK) return st;

  /* 5) 唤醒（AI 保持置位），等待内部振荡器稳定 */
  st = pca_write(PCA9685_REG_MODE1, 0x21U);        /* SLEEP=0, AI=1, ALLCALL=1 */
  if (st != HAL_OK) return st;
  HAL_Delay(1U);

  /* 6) 先给中位，再使能输出。
        注意：把全部 16 路都写成中位，方便排查"舵机插错通道"的情况 */
  s_ready = 1U;
  (void)BSP_Servo_SetAllPulseUs(SERVO_PULSE_MID_US);

  /* 7) 读回确认 */
  if (pca_read(PCA9685_REG_MODE1, &mode1) != HAL_OK) return HAL_ERROR;

  HAL_GPIO_WritePin(SERVO_OE_GPIO_Port, SERVO_OE_Pin, GPIO_PIN_RESET);

  /* 8) 上电自检：单独写 CH0=1500 后逐字节回读，结果留在 RAM 供 SWD 查看
        g_servo_selftest[0..3] = LED0_ON_L / ON_H / OFF_L / OFF_H
        g_servo_selftest[4..6] = MODE1 / MODE2 / PRESCALE
        g_servo_selftest[7]    = 0xA5 魔数（证明自检代码跑到了） */
  HAL_Delay(5U);
  (void)BSP_Servo_SetPulseUs(SERVO_CH_PAN, 1500U);
  HAL_Delay(2U);
  (void)pca_read(0x06U, (uint8_t *)&g_servo_selftest[0]);
  (void)pca_read(0x07U, (uint8_t *)&g_servo_selftest[1]);
  (void)pca_read(0x08U, (uint8_t *)&g_servo_selftest[2]);
  (void)pca_read(0x09U, (uint8_t *)&g_servo_selftest[3]);
  (void)pca_read(0x00U, (uint8_t *)&g_servo_selftest[4]);
  (void)pca_read(0x01U, (uint8_t *)&g_servo_selftest[5]);
  (void)pca_read(0xFEU, (uint8_t *)&g_servo_selftest[6]);
  g_servo_selftest[7] = 0xA5U;

  return HAL_OK;
}

HAL_StatusTypeDef BSP_Servo_SetPulseUs(uint8_t ch, uint16_t pulse_us)
{
  uint8_t buf[4];
  uint16_t counts;

  if (ch >= SERVO_CH_NUM) return HAL_ERROR;

  pulse_us = clamp_pulse(pulse_us);
  s_pulse_us[ch] = pulse_us;

  if (s_ready == 0U) return HAL_BUSY;

  counts = pulse_to_counts(pulse_us);

  buf[0] = 0x00U;                        /* ON_L  = 0 */
  buf[1] = 0x00U;                        /* ON_H  = 0 */
  buf[2] = (uint8_t)(counts & 0xFFU);    /* OFF_L */
  buf[3] = (uint8_t)(counts >> 8);       /* OFF_H */

  return HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDR8,
                           (uint16_t)(PCA9685_REG_LED0 + 4U * ch),
                           I2C_MEMADD_SIZE_8BIT, buf, 4U, PCA9685_I2C_TIMEOUT);
}

/* 把同一个脉宽写到全部 16 路：插错通道也能动，用于排查接线 */
HAL_StatusTypeDef BSP_Servo_SetAllPulseUs(uint16_t pulse_us)
{
  uint8_t  buf[4];
  uint16_t counts = pulse_to_counts(clamp_pulse(pulse_us));

  buf[0] = 0x00U;
  buf[1] = 0x00U;
  buf[2] = (uint8_t)(counts & 0xFFU);
  buf[3] = (uint8_t)(counts >> 8);

  for (uint8_t ch = 0U; ch < PCA9685_CHANNELS; ch++)
  {
    if (HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDR8,
                          (uint16_t)(PCA9685_REG_LED0 + 4U * ch),
                          I2C_MEMADD_SIZE_8BIT, buf, 4U,
                          PCA9685_I2C_TIMEOUT) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef BSP_Servo_SetAngleDeg(uint8_t ch, float angle_deg)
{
  float pulse;

  /* -90 ~ +90 映射到 500 ~ 2500 us */
  pulse = 1500.0f + (angle_deg * 1000.0f / 90.0f);
  if (pulse < (float)SERVO_PULSE_MIN_US) pulse = (float)SERVO_PULSE_MIN_US;
  if (pulse > (float)SERVO_PULSE_MAX_US) pulse = (float)SERVO_PULSE_MAX_US;

  return BSP_Servo_SetPulseUs(ch, (uint16_t)(pulse + 0.5f));
}

uint16_t BSP_Servo_GetPulseUs(uint8_t ch)
{
  if (ch >= SERVO_CH_NUM) return 0U;
  return s_pulse_us[ch];
}

HAL_StatusTypeDef BSP_Servo_SelfTest(uint8_t *prescale, uint8_t *mode1)
{
  HAL_StatusTypeDef st;

  if (HAL_I2C_IsDeviceReady(&hi2c2, PCA9685_ADDR8, 3U, PCA9685_I2C_TIMEOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  st = pca_read(PCA9685_REG_PRESCALE, prescale);
  if (st != HAL_OK) return st;

  st = pca_read(PCA9685_REG_MODE1, mode1);
  return st;
}
