/**
  ******************************************************************************
  * @file    servo_out.h
  * @brief   舵机输出统一出口（PCA9685 或 TIM2 直驱，二选一）
  ******************************************************************************
  */
#ifndef __SERVO_OUT_H
#define __SERVO_OUT_H

#include <stdint.h>
#include "joystick.h"    /* 里面有 SERVO_DRV_PCA9685 开关 */

#if (SERVO_DRV_PCA9685 != 0)
  #include "bsp_servo.h"
  #define SERVO_WRITE(ch, us)  (void)BSP_Servo_SetPulseUs((ch), (us))
#else
  #include "bsp_pwm.h"
  #define SERVO_WRITE(ch, us)  (void)BSP_PWM_SetPulseUs((ch), (us))
#endif

/* 角度 0~180° -> 脉宽 500~2500us */
static inline uint16_t servo_angle_to_pulse(float deg)
{
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  return (uint16_t)(500.0f + deg * (2000.0f / 180.0f) + 0.5f);
}

/* 由「中位 + 单边行程」算出对称的实际限位。
   单边行程会被限制为 min(half, center, 180-center)，
   保证左右对称且永远落在舵机 0~180° 范围内。 */
static inline void servo_calc_limit(float center, float half,
                                    float *lo, float *hi)
{
  float h = half;

  if (h < 0.0f) h = 0.0f;
  if (h > center)          h = center;          /* 不能往小角度方向超 */
  if (h > (180.0f - center)) h = 180.0f - center; /* 不能往大角度方向超 */

  *lo = center - h;
  *hi = center + h;
}

#endif /* __SERVO_OUT_H */
