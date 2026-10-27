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

/* 由中位/单边行程算出的对称限位 */
static float s_pan_lo  = 0.0f;
static float s_pan_hi  = 180.0f;
static float s_tilt_lo = 0.0f;
static float s_tilt_hi = 180.0f;

/* 摄像头注入的误差（TRACK_MODE_SIM 时不使用） */
static volatile float s_inject_ex = 0.0f;
static volatile float s_inject_ey = 0.0f;

/* -------------------------------------------------------------------------- */
void Track_Init(void)
{
  servo_calc_limit(SERVO_PAN_CENTER_DEG,  SERVO_PAN_HALF_DEG,  &s_pan_lo,  &s_pan_hi);
  servo_calc_limit(SERVO_TILT_CENTER_DEG, SERVO_TILT_HALF_DEG, &s_tilt_lo, &s_tilt_hi);

  /* 增量式 PID：输出是"角度增量"，绕机械中位对称限幅 */
  PID_Init(&s_pid_pan,  0.020f, 0.004f, 0.0f,
           s_pan_lo  - SERVO_PAN_CENTER_DEG,
           s_pan_hi  - SERVO_PAN_CENTER_DEG,
           0.25f, 3.0f);
  PID_Init(&s_pid_tilt, 0.020f, 0.004f, 0.0f,
           s_tilt_lo - SERVO_TILT_CENTER_DEG,
           s_tilt_hi - SERVO_TILT_CENTER_DEG,
           0.25f, 3.0f);

  g_pan_deg  = SERVO_PAN_CENTER_DEG;
  g_tilt_deg = SERVO_TILT_CENTER_DEG;
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
    float measured_x = tx - K_DEG2PX * (g_pan_deg  - SERVO_PAN_CENTER_DEG);
    float measured_y = ty - K_DEG2PX * (g_tilt_deg - SERVO_TILT_CENTER_DEG);

    ex = measured_x - (float)(FRAME_W / 2);
    ey = measured_y - (float)(FRAME_H / 2);
  }

  /* 误差 -> 角度增量 -> 以机械中位为中心的绝对角度 */
  g_pan_deg  = SERVO_PAN_CENTER_DEG  + PID_Update(&s_pid_pan,  ex);
  g_tilt_deg = SERVO_TILT_CENTER_DEG + PID_Update(&s_pid_tilt, -ey);

  /* 硬限位：对称限位，任何情况下都不越界 */
  if (g_pan_deg < s_pan_lo)   g_pan_deg = s_pan_lo;
  if (g_pan_deg > s_pan_hi)   g_pan_deg = s_pan_hi;
  if (g_tilt_deg < s_tilt_lo) g_tilt_deg = s_tilt_lo;
  if (g_tilt_deg > s_tilt_hi) g_tilt_deg = s_tilt_hi;

  SERVO_WRITE(0, servo_angle_to_pulse(g_pan_deg));
  SERVO_WRITE(1, servo_angle_to_pulse(g_tilt_deg));
}
