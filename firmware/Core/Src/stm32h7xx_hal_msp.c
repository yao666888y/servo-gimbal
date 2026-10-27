/**
  ******************************************************************************
  * @file    stm32h7xx_hal_msp.c
  * @brief   外设底层初始化（时钟、GPIO 复用、NVIC）
  ******************************************************************************
  */
#include "main.h"

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/* -------------------------------------------------------------------------- */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hi2c->Instance == I2C2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* PB10 = I2C2_SCL, PB11 = I2C2_SDA, 开漏 + 上拉 */
    GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
  }
}

/* -------------------------------------------------------------------------- */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (huart->Instance == USART1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* PA9 = USART1_TX, PA10 = USART1_RX */
    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
}

/* -------------------------------------------------------------------------- */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    __HAL_RCC_TIM6_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    __HAL_RCC_TIM6_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  }
}

/* -------------------------------------------------------------------------- */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC0 = ADC1_INP10，模拟输入 */
    GPIO_InitStruct.Pin  = VBUS_ADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBUS_ADC_GPIO_Port, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_DISABLE();
    HAL_GPIO_DeInit(VBUS_ADC_GPIO_Port, VBUS_ADC_Pin);
  }
}
