/**
  ******************************************************************************
  * @file    pid.h
  * @brief   增量式 PID（适合舵机这类位置型执行器）
  ******************************************************************************
  * 为什么用增量式（速度式）而不是位置式：
  *   舵机本身是位置器件，内部已经有位置环。视觉跟踪时我们希望的是
  *   “每帧给角度加一个修正量”，而不是“每帧重算一个绝对角度”。
  *   增量式天然满足这一点，目标静止时输出会停住（误差为 0 就不再累加）。
  *
  * 输出 out 是"角度增量累计值"，单位就是度，直接加到 90° 上即可。
  *
  * 特性：
  *   - 积分由 out 累加实现，out 限幅 = 抗积分饱和
  *   - 微分项一阶低通，抑制像素噪声
  *   - 误差死区，防止中位抖动
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;          /* 比例 */
  float ki;          /* 积分 */
  float kd;          /* 微分 */

  float out_min;     /* 输出（角度增量）下限，单位度 */
  float out_max;     /* 输出上限 */

  float d_alpha;     /* 微分低通系数 0~1，越小越平滑 */
  float deadband;    /* 误差死区，单位同误差（像素） */

  float e_prev;      /* 上上次误差 */
  float e_prev2;     /* 更早的误差（本实现只保留历史用于调试） */
  float d_filt;      /* 滤波后的微分 */
  float out;         /* 累计输出（角度增量，度） */
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float out_min, float out_max,
               float d_alpha, float deadband);

void  PID_Reset(PID_t *pid);

/* 输入本帧误差，返回累计角度增量（已限幅） */
float PID_Update(PID_t *pid, float error);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
