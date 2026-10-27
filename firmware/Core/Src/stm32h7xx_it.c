/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   中断服务函数
  ******************************************************************************
  */
#include "main.h"
#include "stm32h7xx_it.h"

/* Cortex-M7 内核异常 --------------------------------------------------------*/
void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* 外设中断 ----------------------------------------------------------------*/
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}
