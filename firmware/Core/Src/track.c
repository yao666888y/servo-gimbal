/**
  ******************************************************************************
  * @file    track.c
  * @brief   视觉跟踪闭环 + 虚拟目标验证
  ******************************************************************************
  */
#include <math.h>
#include "track.h"
#include "pid.h"
#include "servo_out.h"

#define PI_F            3.14159265f
#define K_DEG2PX        6.0f     /* 画面像素/度：640 宽约 90° 视场 */
#define SIM_DT_S        0.02f    /* 50Hz */

volatile uint8_t g_track_mode = TRACK_MODE_JOY;
volatile float   g_pan_deg    = 90.0f;
volatile float   g_tilt_deg   = 90.0f;

static PID_t s_pid_pan;
static PID_t s_pid_tilt;
static float s_sim_t = 0.0f;

/* 摄像头注入的误差（TRACK_MODE_SIM 时不使用） */
static volatile float s_inject_ex = 0.0f;
static volatile float s_inject_ey = 0.0f;

/* -------------------------------------------------------------------------- */
void Track_Init(void)
{
  /* 增量式 PID：输出是"角度增量"，限幅即抗积分饱和
     kp/ki 按 50Hz、6px/度 的环路增益整定，先保守一点 */
  PID_Init(&s_pid_pan,  0.020f, 0.004f, 0.0f, -50.0f, 50.0f, 0.25f, 3.0f);
  PID_Init(&s_pid_tilt, 0.020f, 0.004f, 0.0f, -40.0f, 40.0f, 0.25f, 3.0f);

  g_pan_deg  = 90.0f;
  g_tilt_deg = 90.0f;
  s_sim_t    = 0.0f;
  g_track_mode = TRACK_MODE_JOY;
}

void Track_ToggleMode(void)
{
  if (g_track_mode == TRACK_MODE_JOY)
  {
    g_track_mode = TRACK_MODE_SIM;
    PID_Reset(&s_pid_pan);
    PID_Reset(&s_pid_tilt);
    s_sim_t = 0.0f;
  }
  else
  {
    g_track_mode = TRACK_MODE_JOY;
    PID_Reset(&s_pid_pan);
    PID_Reset(&s_pid_tilt);
  }
}

void Track_InjectError(float ex_px, float ey_px)
{
  s_inject_ex = ex_px;
  s_inject_ey = ey_px;
}

/* -------------------------------------------------------------------------- */
void Track_Update(void)
{
  float ex;
  float ey;

  if (g_track_mode != TRACK_MODE_SIM)
  {
    return;   /* 摇杆模式由 joystick.c 的 servo_control() 负责 */
  }

  s_sim_t += SIM_DT_S;

  /* ---- 虚拟目标：在 640x480 画面里做两个不同频率的正弦运动 ----
     接上摄像头后，这一段整体删掉，改用 Track_InjectError() 的实测误差 */
  {
    float tx = (float)(FRAME_W / 2) + 220.0f * sinf(2.0f * PI_F * 0.15f * s_sim_t);
    float ty = (float)(FRAME_H / 2) + 150.0f * sinf(2.0f * PI_F * 0.11f * s_sim_t);

    /* 模拟"摄像机": 云台向右转，目标在画面里向左移 */
    float measured_x = tx - K_DEG2PX * (g_pan_deg  - 90.0f);
    float measured_y = ty - K_DEG2PX * (g_tilt_deg - 90.0f);

    ex = measured_x - (float)(FRAME_W / 2);
    ey = measured_y - (float)(FRAME_H / 2);
  }

  /* 误差 -> 角度增量 -> 绝对角度（叠加机械中位微调） */
  g_pan_deg  = 90.0f + PID_Update(&s_pid_pan,  ex) + SERVO_PAN_TRIM_DEG;
  g_tilt_deg = 90.0f + PID_Update(&s_pid_tilt, -ey) + SERVO_TILT_TRIM_DEG;

  /* 硬限位：任何情况下都不出 0~180° */
  if (g_pan_deg < 0.0f)    g_pan_deg = 0.0f;
  if (g_pan_deg > 180.0f)  g_pan_deg = 180.0f;
  if (g_tilt_deg < 0.0f)   g_tilt_deg = 0.0f;
  if (g_tilt_deg > 180.0f) g_tilt_deg = 180.0f;

  SERVO_WRITE(0, servo_angle_to_pulse(g_pan_deg));
  SERVO_WRITE(1, servo_angle_to_pulse(g_tilt_deg));
}
