/**
  ******************************************************************************
  * @file    vision_link.h
  * @brief   上位机视觉链路（USART1 @921600，二进制帧 + CRC16）
  ******************************************************************************
  * 架构（双目 USB 摄像头方案）：
  *
  *   摄像头 --USB--> PC(OpenCV 检测) --UART--> STM32 -> PID -> PCA9685 -> 舵机
  *
  * 帧格式（小端）：
  *   AA 55 | LEN | CMD | PAYLOAD[LEN-1] | CRC_L CRC_H
  *   LEN   = 1 + payload 长度（含 CMD）
  *   CRC16 = CCITT-FALSE (poly 0x1021, init 0xFFFF)，覆盖 LEN、CMD、PAYLOAD
  *
  * 下行命令：
  *   0x10 TARGET   int16 dx, int16 dy, uint8 valid
  *                 dx/dy = 目标像素坐标 - 画面中心；valid=1 表示本帧检测到目标
  *   0x03 PING     无负载 -> 回 0x83 + 版本 2B
  *
  * 上行：
  *   0x83 PONG     uint8 ver_major, uint8 ver_minor
  *
  * 超时保护：连续 300ms 没有有效 TARGET，云台保持当前角度不动（目标丢失）。
  ******************************************************************************
  */
#ifndef __VISION_LINK_H
#define __VISION_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define VISION_VER_MAJOR   0U
#define VISION_VER_MINOR   2U

/* 启动 USART1 接收中断 */
void Vision_Init(void);

/* 最近一帧是否收到有效目标（30fps 下应一直为 1） */
uint8_t Vision_HasTarget(void);

/* 最近一帧的像素误差（dx 向右为正，dy 向下为正） */
float Vision_GetErrorX(void);
float Vision_GetErrorY(void);

/* 已收到的有效帧计数（调试用） */
uint32_t Vision_GetFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_LINK_H */
