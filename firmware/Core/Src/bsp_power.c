/**
  ******************************************************************************
  * @file    bsp_power.c
  * @brief   V+ 母线电压采样（ADC1_INP10 / PC0，轮询单次转换）
  ******************************************************************************
  * 调用频率建议 ≤ 100Hz，放在主循环里，不要放中断。
  ******************************************************************************
  */
#include "bsp_power.h"

/* 16bit 满量程对应 3300mV（VREF+ = 3.3V） */
#define ADC_FULL_SCALE      65535.0f
#define VREF_MV             3300.0f

volatile uint32_t g_vbus_mv = 0U;

static uint8_t s_ready = 0U;

/* 由 main.c 中的 MX_ADC1_Init() 完成校准后置位 */
void BSP_Power_MarkReady(void)
{
  s_ready = 1U;
}

uint32_t BSP_Power_GetVBusmV(void)
{
  uint32_t raw;
  float    vpin_mv;
  float    vbus_mv;

  if (s_ready == 0U)
  {
    return 0U;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return g_vbus_mv;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 2U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return g_vbus_mv;
  }

  raw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  vpin_mv = ((float)raw * VREF_MV) / ADC_FULL_SCALE;
  vbus_mv = vpin_mv * ((VBUS_DIV_R1_K + VBUS_DIV_R2_K) / VBUS_DIV_R2_K);

  g_vbus_mv = (uint32_t)(vbus_mv + 0.5f);
  return g_vbus_mv;
}
