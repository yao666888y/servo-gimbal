/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层实现
  ******************************************************************************
  * 串口协议（二进制，小端）：
  *   帧： AA 55 | LEN | CMD | PAYLOAD[LEN-1] | CRC16_L CRC16_H
  *        LEN   = 1 + payload 长度（含 CMD）
  *        CRC16 = CCITT-FALSE，多项式 0x1021，初值 0xFFFF
  *        覆盖范围：LEN、CMD、PAYLOAD
  *
  *   下行命令
  *     0x01 SET_ANGLE   int16 pan_x100, int16 tilt_x100   （-90.00 .. +90.00 度）
  *     0x02 SET_PULSE   uint16 pan_us, uint16 tilt_us     （500 .. 2500 us）
  *     0x03 PING        无负载
  *     0x04 GET_STATUS  无负载
  *
  *   上行应答
  *     0x83 PONG        uint8 ver_major, uint8 ver_minor
  *     0x84 STATUS      uint16 pan_us, uint16 tilt_us
  *
  * 控制：50Hz 节拍，目标脉宽按 400us/20ms 斜率逼近，避免舵机猛冲。
  ******************************************************************************
  */
#include "app.h"
#include "main.h"
#include "bsp_servo.h"
#include "bsp_power.h"

#define APP_RX_BUF_SIZE      34U
#define APP_MAX_PAYLOAD      32U

#define APP_SLEW_US_PER_TICK 400U    /* 20ms 内最大脉宽变化 */
#define APP_STATUS_PERIOD_MS 100U    /* 状态上报 10Hz */
#define APP_LED_PERIOD_MS    500U

/* 串口接收状态机 */
typedef enum
{
  RX_WAIT_AA = 0,
  RX_WAIT_55,
  RX_LEN,
  RX_CMD,
  RX_DATA,
  RX_CRC_L,
  RX_CRC_H
} rx_state_t;

static rx_state_t s_rx_state = RX_WAIT_AA;
static uint8_t  s_frame[APP_RX_BUF_SIZE];
static uint8_t  s_data_len;      /* LEN - 1 */
static uint8_t  s_data_idx;
static uint16_t s_crc_rx;
static uint8_t  s_rx_byte;

static uint16_t s_target_us[SERVO_CH_NUM] = {SERVO_PULSE_MID_US, SERVO_PULSE_MID_US};
static uint16_t s_current_us[SERVO_CH_NUM] = {SERVO_PULSE_MID_US, SERVO_PULSE_MID_US};
static uint16_t s_written_us[SERVO_CH_NUM] = {0U, 0U};

static volatile uint8_t  s_tick_flag = 0U;
static volatile uint32_t s_tick_ms = 0U;
static uint32_t s_last_cmd_ms = 0U;
static uint32_t s_last_led_ms = 0U;

/* 应答请求：中断里只置位，发送统一放在主循环，避免 ISR 里阻塞 */
static volatile uint8_t s_ping_req = 0U;
static volatile uint8_t s_status_req = 0U;
static volatile uint8_t  s_all_req = 0U;
static volatile uint16_t s_all_us = SERVO_PULSE_MID_US;
static volatile uint8_t  s_reg_req = 0U;
static volatile uint8_t  s_reg_addr = 0U;
static volatile uint8_t  s_reg_val = 0U;
static volatile uint8_t  s_reg_reply = 0U;
static uint32_t s_last_vbus_ms = 0U;

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

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  uint8_t  buf[APP_RX_BUF_SIZE + 3U];
  uint8_t  n = 0U;
  uint16_t crc;
  uint8_t  len = (uint8_t)(payload_len + 1U);

  buf[n++] = 0xAAU;
  buf[n++] = 0x55U;
  buf[n++] = len;
  buf[n++] = cmd;
  for (uint8_t i = 0U; i < payload_len; i++)
  {
    buf[n++] = payload[i];
  }

  crc = crc16_ccitt(&buf[2], (uint16_t)(len + 1U));   /* LEN + CMD + payload */
  buf[n++] = (uint8_t)(crc & 0xFFU);
  buf[n++] = (uint8_t)(crc >> 8);

  (void)HAL_UART_Transmit(&huart1, buf, n, 10U);
}

static uint16_t clamp_pulse_us(int32_t us)
{
  if (us < (int32_t)SERVO_PULSE_MIN_US) return SERVO_PULSE_MIN_US;
  if (us > (int32_t)SERVO_PULSE_MAX_US) return SERVO_PULSE_MAX_US;
  return (uint16_t)us;
}

static int16_t rd_i16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t rd_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* 角度(-90..+90) -> 脉宽(500..2500)，1000us 对应 90 度 */
static uint16_t angle_to_pulse(const int16_t angle_x100)
{
  int32_t us = 1500 + ((int32_t)angle_x100 * 1000) / 9000;
  return clamp_pulse_us(us);
}

/* -------------------------------------------------------------------------- */
static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  switch (cmd)
  {
    case 0x01U:   /* SET_ANGLE */
      if (payload_len < 4U) break;
      s_target_us[SERVO_CH_PAN]  = angle_to_pulse(rd_i16(&payload[0]));
      s_target_us[SERVO_CH_TILT] = angle_to_pulse(rd_i16(&payload[2]));
      s_last_cmd_ms = s_tick_ms;
      s_status_req = 1U;      /* 每条命令回一帧状态，便于确认 */
      break;

    case 0x02U:   /* SET_PULSE */
      if (payload_len < 4U) break;
      s_target_us[SERVO_CH_PAN]  = clamp_pulse_us((int32_t)rd_u16(&payload[0]));
      s_target_us[SERVO_CH_TILT] = clamp_pulse_us((int32_t)rd_u16(&payload[2]));
      s_last_cmd_ms = s_tick_ms;
      s_status_req = 1U;
      break;

    case 0x03U:   /* PING */
      s_ping_req = 1U;
      break;

    case 0x04U:   /* GET_STATUS */
      s_status_req = 1U;
      break;

    case 0x06U:   /* SET_ALL：同一脉宽写全部 16 路（排查插错通道） */
      if (payload_len < 2U) break;
      s_all_us  = rd_u16(&payload[0]);
      s_all_req = 1U;
      break;

    case 0x07U:   /* READ_REG：读 PCA9685 寄存器（排查用） */
      if (payload_len < 1U) break;
      s_reg_addr = payload[0];
      s_reg_req  = 1U;
      break;

    default:
      break;
  }
}

/* -------------------------------------------------------------------------- */
void App_Init(void)
{
  s_rx_state = RX_WAIT_AA;
  s_tick_flag = 0U;
  s_tick_ms = 0U;

  for (uint8_t i = 0U; i < SERVO_CH_NUM; i++)
  {
    s_target_us[i]  = SERVO_PULSE_MID_US;
    s_current_us[i] = SERVO_PULSE_MID_US;
    s_written_us[i] = 0U;
  }

  HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);

  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);

  s_last_cmd_ms = 0U;
  s_ping_req = 0U;
  s_status_req = 0U;
  s_last_vbus_ms = 0U;
  s_last_led_ms = 0U;
}

void App_Tick50Hz(void)
{
  s_tick_ms += 20U;
  s_tick_flag = 1U;
}

/* 目标脉宽按斜率逼近，并只在变化时写 I2C，减少总线占用 */
static void control_step(void)
{
  for (uint8_t ch = 0U; ch < SERVO_CH_NUM; ch++)
  {
    int32_t diff = (int32_t)s_target_us[ch] - (int32_t)s_current_us[ch];

    if (diff > (int32_t)APP_SLEW_US_PER_TICK)
    {
      diff = (int32_t)APP_SLEW_US_PER_TICK;
    }
    else if (diff < -(int32_t)APP_SLEW_US_PER_TICK)
    {
      diff = -(int32_t)APP_SLEW_US_PER_TICK;
    }

    s_current_us[ch] = clamp_pulse_us((int32_t)s_current_us[ch] + diff);

    if (s_current_us[ch] != s_written_us[ch])
    {
      if (BSP_Servo_SetPulseUs(ch, s_current_us[ch]) == HAL_OK)
      {
        s_written_us[ch] = s_current_us[ch];
      }
    }
  }
}

void App_Loop(void)
{
  /* 50Hz 控制节拍（由 TIM6 中断置位，I2C 写放在主循环上下文） */
  if (s_tick_flag != 0U)
  {
    s_tick_flag = 0U;
    control_step();
  }

  /* SET_ALL：全部 16 路写同一脉宽 */
  if (s_all_req != 0U)
  {
    s_all_req = 0U;
    (void)BSP_Servo_SetAllPulseUs(s_all_us);
    s_status_req = 1U;
  }

  /* READ_REG：读寄存器后回 0x87 */
  if (s_reg_req != 0U)
  {
    uint8_t v = 0xEEU;

    s_reg_req = 0U;
    if (BSP_Servo_ReadReg((uint8_t)s_reg_addr, &v) != HAL_OK)
    {
      v = 0xEEU;    /* 读失败标记 */
    }
    s_reg_val   = v;
    s_reg_reply = 1U;
  }

  if (s_reg_reply != 0U)
  {
    uint8_t tx[2];

    s_reg_reply = 0U;
    tx[0] = s_reg_addr;
    tx[1] = s_reg_val;
    send_frame(0x87U, tx, 2U);
  }

  /* 应答（统一在主循环发送，避免在中断里阻塞） */
  if (s_ping_req != 0U)
  {
    uint8_t tx[2];

    s_ping_req = 0U;
    tx[0] = APP_VERSION_MAJOR;
    tx[1] = APP_VERSION_MINOR;
    send_frame(0x83U, tx, 2U);
  }

  if (s_status_req != 0U)
  {
    uint8_t  tx[6];
    uint32_t vbus = g_vbus_mv;

    s_status_req = 0U;
    tx[0] = (uint8_t)(s_current_us[SERVO_CH_PAN] & 0xFFU);
    tx[1] = (uint8_t)(s_current_us[SERVO_CH_PAN] >> 8);
    tx[2] = (uint8_t)(s_current_us[SERVO_CH_TILT] & 0xFFU);
    tx[3] = (uint8_t)(s_current_us[SERVO_CH_TILT] >> 8);
    tx[4] = (uint8_t)(vbus & 0xFFU);          /* V+ 母线电压 mV */
    tx[5] = (uint8_t)((vbus >> 8) & 0xFFU);
    send_frame(0x84U, tx, 6U);
  }

  /* 10Hz 采样 V+ 母线电压（用于判断舵机电源/电流） */
  if ((s_tick_ms - s_last_vbus_ms) >= 100U)
  {
    s_last_vbus_ms = s_tick_ms;
    (void)BSP_Power_GetVBusmV();
  }

  /* 心跳灯 */
  if ((s_tick_ms - s_last_led_ms) >= APP_LED_PERIOD_MS)
  {
    s_last_led_ms = s_tick_ms;
    HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
  }
}

void App_OnRxByte(uint8_t byte)
{
  switch (s_rx_state)
  {
    case RX_WAIT_AA:
      if (byte == 0xAAU) s_rx_state = RX_WAIT_55;
      break;

    case RX_WAIT_55:
      s_rx_state = (byte == 0x55U) ? RX_LEN : RX_WAIT_AA;
      break;

    case RX_LEN:
      if ((byte == 0U) || (byte > (APP_MAX_PAYLOAD + 1U)))
      {
        s_rx_state = RX_WAIT_AA;
      }
      else
      {
        s_frame[0] = byte;
        s_data_len = (uint8_t)(byte - 1U);
        s_data_idx = 0U;
        s_rx_state = RX_CMD;
      }
      break;

    case RX_CMD:
      s_frame[1] = byte;
      s_rx_state = (s_data_len > 0U) ? RX_DATA : RX_CRC_L;
      break;

    case RX_DATA:
      s_frame[2U + s_data_idx] = byte;
      s_data_idx++;
      if (s_data_idx >= s_data_len) s_rx_state = RX_CRC_L;
      break;

    case RX_CRC_L:
      s_crc_rx = byte;
      s_rx_state = RX_CRC_H;
      break;

    case RX_CRC_H:
      s_crc_rx |= (uint16_t)((uint16_t)byte << 8);
      if (s_crc_rx == crc16_ccitt(s_frame, (uint16_t)(1U + 1U + s_data_len)))
      {
        handle_frame(s_frame[1], &s_frame[2], s_data_len);
      }
      s_rx_state = RX_WAIT_AA;
      break;

    default:
      s_rx_state = RX_WAIT_AA;
      break;
  }
}

/* -------------------------------------------------------------------------- */
/* HAL 回调                                                                   */
/* -------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    App_Tick50Hz();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    App_OnRxByte(s_rx_byte);
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
  }
}
