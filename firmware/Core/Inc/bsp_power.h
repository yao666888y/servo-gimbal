/**
  ******************************************************************************
  * @file    bsp_power.h
  * @brief   V+ 母线电压监测（PC0 / ADC1_INP10，电阻分压）
  ******************************************************************************
  * 硬件：
  *
  *   V+ (6~8.4V) ──[ R1 = 10k ]──┬── PC0 (ADC1_INP10)
  *                               │
  *                          [ R2 = 3.3k ]
  *                               │
  *                              GND
  *
  *   Vpin = V+ * R2 / (R1 + R2)      → 7.4V 时约 1.84V
  *   V+   = Vpin * (R1 + R2) / R2    → 量程上限 13.3V（16bit 满量程）
  *
  * 用途：判断舵机电源是否上电、是否在吃电流（动作时 V+ 会跌落 100~300mV）
  ******************************************************************************
  */
#ifndef __BSP_POWER_H
#define __BSP_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* 分压电阻（kΩ），换电阻只改这两个值 */
#define VBUS_DIV_R1_K   10.0f
#define VBUS_DIV_R2_K   3.3f

/* 返回 V+ 母线电压，单位 mV；ADC 未就绪时返回 0 */
uint32_t BSP_Power_GetVBusmV(void);

/* 由 ADC 初始化完成后调用，使能采样 */
void BSP_Power_MarkReady(void);

/* 最近一次采样值（供 SWD 调试直接读） */
extern volatile uint32_t g_vbus_mv;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_POWER_H */
