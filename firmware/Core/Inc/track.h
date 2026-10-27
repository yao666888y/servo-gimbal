/**
  ******************************************************************************
  * @file    track.h
  * @brief   视觉跟踪闭环（当前用"虚拟目标"验证 PID，摄像头到货后替换误差源）
  ******************************************************************************
  * 数据流：
  *
  *   摇杆模式:   ADC -> 角度                （手动，已可用）
  *   跟踪模式:   目标像素 -> 误差 -> PID -> 角度增量 -> 角度
  *
  * 摄像头接入点只有一个：把 Track_Update() 里的 ex/ey 换成实测误差：
  *
  *     ex = (float)(target_px - FRAME_W/2);
  *     ey = (float)(target_py - FRAME_H/2);
  *
  * 其余（PID、限幅、脉宽换算、舵机输出）完全复用。
  ******************************************************************************
  */
#ifndef __TRACK_H
#define __TRACK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 画面分辨率（与实际摄像头一致，当前按 640x480 假设） */
#define FRAME_W   640
#define FRAME_H   480

#define TRACK_MODE_JOY   0U   /* 摇杆手动 */
#define TRACK_MODE_SIM   1U   /* 虚拟目标 + PID 闭环 */
#define TRACK_MODE_CAM   2U   /* 上位机视觉（串口注入误差） */

/* 当前工作模式（长按摇杆帽切换） */
extern volatile uint8_t g_track_mode;

/* 当前云台角度，调试观察用 */
extern volatile float g_pan_deg;
extern volatile float g_tilt_deg;

void Track_Init(void);
void Track_Update(void);        /* 50Hz 调用 */
void Track_ToggleMode(void);    /* 摇杆手动 <-> 虚拟目标跟踪 */

/* 供摄像头模块调用：注入本帧像素误差（相对画面中心），并切到 CAM 模式 */
void Track_InjectError(float ex_px, float ey_px);
void Track_SetTarget(float ex_px, float ey_px);
uint8_t Track_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRACK_H */
