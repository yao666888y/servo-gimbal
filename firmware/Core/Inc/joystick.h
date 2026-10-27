/**
  ******************************************************************************
  * @file    joystick.h
  * @brief   摇杆输入（ADC1 DMA 双通道）+ SW 按键 + 云台控制
  ******************************************************************************
  * 硬件（KY-023 类摇杆模块）：
  *   VCC -> 3.3V
  *   GND -> GND
  *   URX -> PA2 (ADC1_INP14)
  *   URY -> PA3 (ADC1_INP15)
  *   SW  -> PB1 (GPIO 输入，内部上拉，按下为低)
  *
  * 数据流：
  *   ADC(0~4095) -> 控制量(-100~100) -> 死区(±10) -> 一阶滤波
  *   -> 角度(0~180°, 90° 中位) -> 脉宽(500~2500us) -> TIM2 CCR
  ******************************************************************************
  */
#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* 摇杆中位（12bit ADC） */
#define JOY_ADC_CENTER   2048

/* 舵机输出通道选择：
 *   1 = PCA9685（I2C2，舵机保持接在驱动板上不动）—— 默认
 *   0 = TIM2 直驱（PA0/PA1），需要把舵机信号线改接到 MCU
 */
#define SERVO_DRV_PCA9685  1

/* 死区：|控制量| < 10 视为 0，抑制中位抖动 */
#define JOY_DEADBAND     10
/* 满量程控制量 */
#define JOY_FULL_SCALE   100

/* 上电自检：ADC 值必须落在这个区间才认为摇杆接上了
   （H7 模拟输入悬空一般读到 100~300，会被判为"未接"，从而锁在中位） */
#define JOY_RAW_MIN      500
#define JOY_RAW_MAX      3600

/* 方向反转开关：装好后发现方向相反就改这里 */
#define JOY_X_INVERT     0
#define JOY_Y_INVERT     1

/* 输出：-100 ~ +100，已滤波、已过死区 */
extern volatile int16_t  g_stick_x;
extern volatile int16_t  g_stick_y;

/* 原始 ADC 值（调试用） */
extern volatile uint16_t g_joy_raw[2];

/* 本周期检测到 SW 按下（下降沿），由 servo_control 消费 */
extern volatile uint8_t  g_joy_key;

/* 长按（>800ms）事件：主循环用它切换 摇杆/跟踪 模式 */
extern volatile uint8_t  g_joy_long;

void Joystick_Init(void);   /* 启动 ADC + DMA 连续转换 */
void joystick_read(void);   /* 读 DMA 缓冲 -> 映射 -> 死区 -> 滤波 */
void key_scan(void);        /* 扫描 SW 下降沿 */
void servo_control(void);   /* 控制量 -> 角度 -> 脉宽，含回中逻辑 */

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
