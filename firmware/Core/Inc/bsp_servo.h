/**
  ******************************************************************************
  * @file    bsp_servo.h
  * @brief   PCA9685 16 路舵机驱动板（I2C2）
  ******************************************************************************
  * 时序要点：
  *   1) OE 上电为高（输出关断）
  *   2) SLEEP=1 状态下写 PRESCALE = 121 -> 50.03Hz
  *   3) 清 SLEEP，等 500us，发 RESTART
  *   4) MODE2 = 0x04（OUTDRV 推挽）
  *   5) CH0/CH1 先写 1500us 中位，再拉低 OE 使能
  ******************************************************************************
  */
#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* 通道分配 */
#define SERVO_CH_PAN     0U   /* 水平 */
#define SERVO_CH_TILT    1U   /* 俯仰 */
#define SERVO_CH_NUM     2U
#define PCA9685_CHANNELS 16U  /* 板上总通道数 */

/* 脉宽范围（us） */
#define SERVO_PULSE_MIN_US   500U
#define SERVO_PULSE_MAX_US   2500U
#define SERVO_PULSE_MID_US   1500U

/* PCA9685 I2C 7bit 地址（A0-A5 全 0） */
#define PCA9685_I2C_ADDR_7B  0x40U

HAL_StatusTypeDef BSP_Servo_Init(void);

/* 直接写脉宽，内部自动限幅到 [500, 2500] */
HAL_StatusTypeDef BSP_Servo_SetPulseUs(uint8_t ch, uint16_t pulse_us);

/* 同一脉宽写全部 16 路（排查插错通道用） */
HAL_StatusTypeDef BSP_Servo_SetAllPulseUs(uint16_t pulse_us);

/* 读 PCA9685 任意寄存器（调试用） */
HAL_StatusTypeDef BSP_Servo_ReadReg(uint8_t reg, uint8_t *value);

/* 角度(-90)~(+90) -> 脉宽 -> 寄存器，内部限幅 */
HAL_StatusTypeDef BSP_Servo_SetAngleDeg(uint8_t ch, float angle_deg);

uint16_t BSP_Servo_GetPulseUs(uint8_t ch);

/* 读回 PRESCALE 与 MODE1，用于自检（返回 HAL_OK 表示器件在线） */
HAL_StatusTypeDef BSP_Servo_SelfTest(uint8_t *prescale, uint8_t *mode1);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_H */
