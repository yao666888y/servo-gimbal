/**
  ******************************************************************************
  * @file    vision_link.c
  * @brief   上位机视觉链路：USART1 收帧 -> 解析 -> 喂给 Track
  ******************************************************************************
  */
#include "vision_link.h"
#include "main.h"
#include "track.h"

#define VL_RX_BUF      32U
#define VL_TIMEOUT_MS  300U

typedef enum
{
  VL_AA = 0,
  VL_55,
  VL_LEN,
  VL_CMD,
  VL_DATA,
  VL_CRC_L,
  VL_CRC_H
} vl_state_t;

static vl_state_t s_state = VL_AA;
static uint8_t  s_frame[VL_RX_BUF];
static uint8_t  s_len;          /* 收到的 LEN（cmd + payload） */
static uint8_t  s_idx;
static uint8_t  s_rx_byte;
static uint16_t s_crc_rx;

static volatile float   s_ex = 0.0f;
static volatile float   s_ey = 0.0f;
static volatile uint8_t s_valid = 0U;
static volatile uint32_t s_last_ms = 0U;
static volatile uint32_t s_frames = 0U;

/* -------------------------------------------------------------------------- */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint8_t b = 0U; b < 8U; b++)
    {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void send_pong(void)
{
  uint8_t buf[8];
  uint8_t n = 0U;
  uint16_t crc;

  buf[n++] = 0xAAU;
  buf[n++] = 0x55U;
  buf[n++] = 0x03U;                  /* LEN = cmd + 2B 版本 */
  buf[n++] = 0x83U;
  buf[n++] = VISION_VER_MAJOR;
  buf[n++] = VISION_VER_MINOR;

  crc = crc16_ccitt(&buf[2], 4U);
  buf[n++] = (uint8_t)(crc & 0xFFU);
  buf[n++] = (uint8_t)(crc >> 8);

  (void)HAL_UART_Transmit(&huart1, buf, n, 5U);
}

static int16_t rd_i16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* -------------------------------------------------------------------------- */
static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
  if (cmd == 0x10U)                    /* TARGET */
  {
    if (len < 4U) return;

    s_ex = (float)rd_i16(&payload[0]);
    s_ey = (float)rd_i16(&payload[2]);
    s_valid = (len >= 5U) ? ((payload[4] & 0x01U) ? 1U : 0U) : 1U;

    if (s_valid != 0U)
    {
      s_last_ms = HAL_GetTick();
      s_frames++;
      Track_SetTarget(s_ex, s_ey);     /* 交给 PID，并切到摄像头模式 */
    }
  }
  else if (cmd == 0x03U)               /* PING */
  {
    send_pong();
  }
}

/* -------------------------------------------------------------------------- */
void Vision_Init(void)
{
  s_state  = VL_AA;
  s_valid  = 0U;
  s_frames = 0U;

  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

uint8_t Vision_HasTarget(void)
{
  if (s_valid == 0U) return 0U;
  return ((HAL_GetTick() - s_last_ms) < VL_TIMEOUT_MS) ? 1U : 0U;
}

float Vision_GetErrorX(void) { return s_ex; }
float Vision_GetErrorY(void) { return s_ey; }
uint32_t Vision_GetFrameCount(void) { return s_frames; }

/* -------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t b;

  if (huart->Instance != USART1) return;

  b = s_rx_byte;

  switch (s_state)
  {
    case VL_AA:
      if (b == 0xAAU) s_state = VL_55;
      break;

    case VL_55:
      s_state = (b == 0x55U) ? VL_LEN : VL_AA;
      break;

    case VL_LEN:
      if ((b == 0U) || (b > (VL_RX_BUF - 2U)))
      {
        s_state = VL_AA;
      }
      else
      {
        s_frame[0] = b;
        s_len = b;
        s_idx = 0U;
        s_state = VL_CMD;
      }
      break;

    case VL_CMD:
      s_frame[1] = b;
      s_state = (s_len > 1U) ? VL_DATA : VL_CRC_L;
      break;

    case VL_DATA:
      s_frame[2U + s_idx] = b;
      s_idx++;
      if (s_idx >= (uint8_t)(s_len - 1U)) s_state = VL_CRC_L;
      break;

    case VL_CRC_L:
      s_crc_rx = b;
      s_state = VL_CRC_H;
      break;

    case VL_CRC_H:
      s_crc_rx |= (uint16_t)((uint16_t)b << 8);
      if (s_crc_rx == crc16_ccitt(s_frame, (uint16_t)(s_len + 1U)))
      {
        handle_frame(s_frame[1], &s_frame[2], (uint8_t)(s_len - 1U));
      }
      s_state = VL_AA;
      break;

    default:
      s_state = VL_AA;
      break;
  }

  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}
