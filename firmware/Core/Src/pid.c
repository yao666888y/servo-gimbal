/**
  ******************************************************************************
  * @file    pid.c
  * @brief   增量式 PID 实现
  ******************************************************************************
  */
#include "pid.h"

static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float fabsf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max,
              float d_alpha, float deadband)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->out_min = out_min;
  pid->out_max = out_max;
  pid->d_alpha = clampf(d_alpha, 0.0f, 1.0f);
  pid->deadband = (deadband < 0.0f) ? 0.0f : deadband;
  PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
  pid->e_prev  = 0.0f;
  pid->e_prev2 = 0.0f;
  pid->d_filt  = 0.0f;
  pid->out     = 0.0f;
}

float PID_Update(PID_t *pid, float error)
{
  float d_raw;
  float delta;

  /* 死区：小幅误差不响应，避免中位高频抖动 */
  if (fabsf_local(error) < pid->deadband)
  {
    error = 0.0f;
  }

  /* 微分项一阶低通，抑制像素噪声放大 */
  d_raw = error - pid->e_prev;
  pid->d_filt += pid->d_alpha * (d_raw - pid->d_filt);

  /* 增量式：Δu = Kp·Δe + Ki·e + Kd·(Δe 滤波) */
  delta = (pid->kp * (error - pid->e_prev)) +
          (pid->ki * error) +
          (pid->kd * pid->d_filt);

  pid->e_prev2 = pid->e_prev;
  pid->e_prev  = error;

  /* 累加并限幅：限幅本身即抗积分饱和（windup） */
  pid->out = clampf(pid->out + delta, pid->out_min, pid->out_max);

  return pid->out;
}
