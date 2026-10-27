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

#endif /* __SERVO_OUT_H */
