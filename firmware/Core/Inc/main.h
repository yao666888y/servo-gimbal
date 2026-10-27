/**
  ******************************************************************************
  * @file    main.h
  * @brief   板级定义与全局句柄（Phase 1：PCA9685 云台 + 串口）
  ******************************************************************************
  * 引脚分配（与 docs/CubeMX_PinConfig.md §0.5 Phase 1 一致）
  *   PB10  I2C2_SCL  -> PCA9685 SCL  (AF4, 开漏, 上拉)
  *   PB11  I2C2_SDA  -> PCA9685 SDA  (AF4, 开漏, 上拉)
  *   PD6   GPIO_OUT  -> PCA9685 OE   (低有效, 上电默认高=关输出)
  *   PB0   GPIO_OUT  -> LED_RUN
  *   PA9   USART1_TX (AF7, 921600 8N1)
  *   PA10  USART1_RX (AF7)
  *   PA13/PA14 SWD
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* 板级引脚 ----------------------------------------------------------------*/
#define LED_RUN_Pin           GPIO_PIN_0
#define LED_RUN_GPIO_Port     GPIOB

#define SERVO_OE_Pin          GPIO_PIN_6
#define SERVO_OE_GPIO_Port    GPIOD

#define VBUS_ADC_Pin          GPIO_PIN_0
#define VBUS_ADC_GPIO_Port    GPIOC

/* 全局外设句柄 ------------------------------------------------------------*/
extern I2C_HandleTypeDef  hi2c2;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef  htim6;
extern ADC_HandleTypeDef  hadc1;

/* 公共函数 ----------------------------------------------------------------*/
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
