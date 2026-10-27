/**
  ******************************************************************************
  * @file    bsp_pwm.h
  * @brief   舵机 PWM 输出（TIM2 CH1/CH2，50Hz，1us 分辨率）
  ******************************************************************************
  * 引脚：PA0 = TIM2_CH1（水平 Pan）
  *       PA1 = TIM2_CH2（垂直 Tilt）
  *
  * TIM2 内核时钟 240MHz，PSC=239 -> 1MHz 计数，ARR=19999 -> 20ms/50Hz
  * 脉宽 500~2500us 对应 0~180°
  ******************************************************************************
  */
#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

#define SERVO_PAN            0U
#define SERVO_TILT           1U
#define SERVO_NUM            2U

#define SERVO_PULSE_MIN_US   500U
#define SERVO_PULSE_MAX_US   2500U
#define SERVO_PULSE_MID_US   1500U

#define SERVO_ANGLE_MIN_DEG  0.0f
#define SERVO_ANGLE_MAX_DEG  180.0f
#define SERVO_ANGLE_MID_DEG  90.0f

/* 启动 TIM2 双通道 PWM，初始输出中位 1500us */
void BSP_PWM_Init(void);

/* 直接写脉宽，内部限幅到 [500, 2500] */
void BSP_PWM_SetPulseUs(uint8_t ch, uint16_t pulse_us);

/* 角度 0~180° -> 脉宽，内部限幅 */
void BSP_PWM_SetAngle(uint8_t ch, float angle_deg);

uint16_t BSP_PWM_GetPulseUs(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
