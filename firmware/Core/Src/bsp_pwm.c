/**
  ******************************************************************************
  * @file    bsp_pwm.c
  * @brief   TIM2 双通道 50Hz 舵机 PWM
  ******************************************************************************
  */
#include "bsp_pwm.h"

static uint16_t s_pulse_us[SERVO_NUM] = {SERVO_PULSE_MID_US, SERVO_PULSE_MID_US};

static uint16_t clamp_pulse(uint16_t us)
{
  if (us < SERVO_PULSE_MIN_US) return SERVO_PULSE_MIN_US;
  if (us > SERVO_PULSE_MAX_US) return SERVO_PULSE_MAX_US;
  return us;
}

void BSP_PWM_Init(void)
{
  /* 先给中位，避免上电瞬间舵机打到极限 */
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  BSP_PWM_SetPulseUs(SERVO_PAN,  SERVO_PULSE_MID_US);
  BSP_PWM_SetPulseUs(SERVO_TILT, SERVO_PULSE_MID_US);
}

void BSP_PWM_SetPulseUs(uint8_t ch, uint16_t pulse_us)
{
  if (ch >= SERVO_NUM) return;

  pulse_us = clamp_pulse(pulse_us);
  s_pulse_us[ch] = pulse_us;

  __HAL_TIM_SET_COMPARE(&htim2,
                        (ch == SERVO_PAN) ? TIM_CHANNEL_1 : TIM_CHANNEL_2,
                        pulse_us);
}

void BSP_PWM_SetAngle(uint8_t ch, float angle_deg)
{
  float us;

  if (angle_deg < SERVO_ANGLE_MIN_DEG) angle_deg = SERVO_ANGLE_MIN_DEG;
  if (angle_deg > SERVO_ANGLE_MAX_DEG) angle_deg = SERVO_ANGLE_MAX_DEG;

  us = (float)SERVO_PULSE_MIN_US +
       (angle_deg / SERVO_ANGLE_MAX_DEG) *
       (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);

  BSP_PWM_SetPulseUs(ch, (uint16_t)(us + 0.5f));
}

uint16_t BSP_PWM_GetPulseUs(uint8_t ch)
{
  if (ch >= SERVO_NUM) return 0U;
  return s_pulse_us[ch];
}
