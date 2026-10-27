/**
  ******************************************************************************
  * @file    main.c
  * @brief   Phase 1 主程序：480MHz + PCA9685 云台 + 串口协议
  ******************************************************************************
  * 时钟树（不依赖外部晶振，任何 H743 板子都能直接跑）：
  *   HSI 64MHz / PLLM(16) = 4MHz  -> PLLN(480) = VCO 960MHz
  *   PLLP = 2 -> SYSCLK 480MHz；PLLQ = 8 -> 120MHz
  *   D1CPRE /1 -> CPU 480MHz；HPRE /2 -> AXI 240MHz
  *   APB1/2/3/4 /2 -> 120MHz；定时器内核 240MHz
  *   VOS0 + Flash 4WS
  *
  * 改用外部晶振时只改三处：RCC_OSCILLATORTYPE_HSE / PLL.PLLM / PLL.PLLSource
  *   25MHz 晶振：PLLM = 5；8MHz 晶振：PLLM = 4
  ******************************************************************************
  */
#include "main.h"
#include "app.h"          /* 保留：串口协议（当前未启用） */
#include "bsp_servo.h"    /* 保留：PCA9685 驱动（当前未启用） */
#include "bsp_power.h"
#include "bsp_pwm.h"      /* TIM2 双路舵机 PWM */
#include "joystick.h"     /* 摇杆 + SW 按键 */
#include "track.h"        /* PID 视觉跟踪 */

I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim6;
TIM_HandleTypeDef  htim2;
ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;

static void MPU_Config(void);
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
#if (SERVO_DRV_PCA9685 == 0)
static void MX_TIM2_Init(void);
#endif
static void MX_ADC1_Init(void);

/**
  * @brief  摇杆手动控制演示
  *
  * 硬件接线（默认：舵机保持接在 PCA9685 驱动板上）：
  *   舵机 Pan/Tilt -> PCA9685 CH0/CH1（原有接线不动）
  *   I2C2: PB10=SCL  PB11=SDA  PD6=OE（原有接线不动）
  *   摇杆 URX -> PA2 (ADC1_INP14)        摇杆 VCC -> 3.3V
  *   摇杆 URY -> PA3 (ADC1_INP15)        摇杆 GND -> GND
  *   摇杆 SW  -> PB1 (内部上拉，按下为低)
  *
  * 若改用 TIM2 直驱，把 joystick.h 的 SERVO_DRV_PCA9685 改成 0，
  * 并把舵机信号线改接到 PA0(TIM2_CH1) / PA1(TIM2_CH2)。
  *
  * 控制链路（50Hz）：
  *   ADC DMA -> joystick_read() 映射/死区/滤波 -> key_scan() -> servo_control()
  */
int main(void)
{
  uint32_t now;
  uint32_t last_ctrl;
  uint32_t last_led;

  HAL_Init();

  MPU_Config();

  /* 打开 I/D Cache（MPU 先把 DMA 区设成 Non-Cacheable，顺序不能反） */
  SCB_EnableICache();
  SCB_EnableDCache();

  SystemClock_Config();

  MX_GPIO_Init();          /* LED / SW 按键 / PCA9685 OE */
  MX_ADC1_Init();          /* PA2/PA3 双通道 + DMA */

  /* 串口保留：需要调试输出时可以直接用 huart1 */
  MX_USART1_UART_Init();
  MX_TIM6_Init();          /* 保留：50Hz 节拍源，这里不启动（主循环用 SysTick） */

#if (SERVO_DRV_PCA9685 != 0)
  /* 舵机接在 PCA9685 驱动板上：原有接线完全不用动 */
  MX_I2C2_Init();
  (void)BSP_Servo_Init();  /* 写寄存器并把 OE 拉低使能输出 */
#else
  /* TIM2 直驱：舵机信号线需改接到 PA0/PA1 */
  MX_TIM2_Init();
  BSP_PWM_Init();          /* 两路先输出 1500us 中位 */
#endif

  Joystick_Init();         /* 启动 ADC 连续转换 + 循环 DMA */
  Track_Init();            /* PID + 虚拟目标跟踪 */

  last_ctrl = HAL_GetTick();
  last_led  = HAL_GetTick();

  while (1)
  {
    now = HAL_GetTick();

    /* 50Hz 控制节拍（20ms） */
    if ((now - last_ctrl) >= 20U)
    {
      last_ctrl = now;

      joystick_read();     /* ADC -> -100~100，死区 + 滤波 */
      key_scan();          /* SW 下降沿 + 长按 */

      /* 长按摇杆帽：切换 摇杆手动 / 虚拟目标跟踪 */
      if (g_joy_long != 0U)
      {
        g_joy_long = 0U;
        Track_ToggleMode();
      }

      if (g_track_mode == TRACK_MODE_SIM)
      {
        Track_Update();    /* PID 闭环：虚拟目标 -> 误差 -> 角度 */
      }
      else
      {
        servo_control();   /* 手动：-100~100 -> 角度 -> 脉宽，含回中 */
      }
    }

    /* 心跳灯 1Hz */
    if ((now - last_led) >= 500U)
    {
      last_led = now;
      HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
  }
}

/* -------------------------------------------------------------------------- */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  /* Region 0: AXI SRAM 前 256KB，Normal Non-Cacheable Shareable
     —— 供后续 DCMI / UART DMA 缓冲区使用，CPU 访问不会被 cache 脏数据影响 */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = 0x24000000U;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.SubRegionDisable = 0x00U;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* -------------------------------------------------------------------------- */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* 1) 供电与电压等级：H743（LQFP100 / LQFP144 相同）用 LDO，480MHz 必须 VOS0
        （H7 的 PWR 时钟默认开启，无需 __HAL_RCC_PWR_CLK_ENABLE） */
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == 0U)
  {
  }

  /* 2) PLL1: HSI 64MHz -> 4MHz 参考 -> VCO 960MHz */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16U;   /* 64MHz / 16 = 4MHz 参考 */
  RCC_OscInitStruct.PLL.PLLN            = 240U;  /* 4MHz x 240 = 960MHz VCO（上限） */
  RCC_OscInitStruct.PLL.PLLP            = 2U;
  RCC_OscInitStruct.PLL.PLLQ            = 8U;
  RCC_OscInitStruct.PLL.PLLR            = 2U;
  RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_2;   /* 4-8MHz */
  RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN        = 0U;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 3) 总线分频 */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                      RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;    /* CPU 480MHz */
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;      /* AXI 240MHz */
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;      /* 120MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;      /* 120MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;      /* 120MHz */
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;      /* 120MHz */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* -------------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* PB0: LED_RUN */
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = LED_RUN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RUN_GPIO_Port, &GPIO_InitStruct);

  /* PD6: SERVO_OE，初始高 = 输出关断
     （TIM2 直驱时不用 PCA9685 输出；保持关断可避免两路信号打架） */
  HAL_GPIO_WritePin(SERVO_OE_GPIO_Port, SERVO_OE_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = SERVO_OE_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_OE_GPIO_Port, &GPIO_InitStruct);

  /* PB1: 摇杆 SW，内部上拉，按下读到低电平 */
  GPIO_InitStruct.Pin  = JOY_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(JOY_SW_GPIO_Port, &GPIO_InitStruct);
}

/* -------------------------------------------------------------------------- */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance              = I2C2;
  /* 100kHz @ I2CCLK 120MHz：PRESC=11(12 分频,100ns), SCLDEL=4, SDADEL=2,
     SCLH=52(5.3us), SCLL=47(4.8us) -> ~99kHz */
  hi2c2.Init.Timing           = 0xB042342FU;
  hi2c2.Init.OwnAddress1      = 0U;
  hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2      = 0U;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* -------------------------------------------------------------------------- */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 921600U;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* -------------------------------------------------------------------------- */
static void MX_TIM6_Init(void)
{
  htim6.Instance               = TIM6;
  htim6.Init.Prescaler         = 239U;      /* 240MHz / 240 = 1MHz */
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.Period            = 19999U;    /* 20ms -> 50Hz */
  htim6.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* -------------------------------------------------------------------------- */
#if (SERVO_DRV_PCA9685 == 0)
/* TIM2 CH1/CH2：50Hz 舵机 PWM
   TIM2 内核 240MHz -> PSC=239 得 1MHz（1 计数 = 1us）
   ARR=19999 -> 20ms；CCR 500~2500 -> 0.5~2.5ms */
static void MX_TIM2_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 239U;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 19999U;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = SERVO_PULSE_MID_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif /* SERVO_DRV_PCA9685 == 0 */

/* -------------------------------------------------------------------------- */
/* ADC1 双通道扫描 + 循环 DMA：PA2(INP14)=URX，PA3(INP15)=URY */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                      = ADC1;
  hadc1.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;  /* 120/4 = 30MHz */
  hadc1.Init.Resolution               = ADC_RESOLUTION_12B;        /* 0~4095 */
  hadc1.Init.ScanConvMode             = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection             = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait         = DISABLE;
  hadc1.Init.ContinuousConvMode       = ENABLE;                    /* 连续转换 */
  hadc1.Init.NbrOfConversion          = 2U;
  hadc1.Init.DiscontinuousConvMode    = DISABLE;
  hadc1.Init.NbrOfDiscConversion      = 0U;
  hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode         = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.SamplingTime           = ADC_SAMPLETIME_810CYCLES_5;   /* 摇杆是 10k 电位器，长采样 */
  sConfig.SingleDiff             = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber           = ADC_OFFSET_NONE;
  sConfig.Offset                 = 0U;
  sConfig.OffsetSignedSaturation = DISABLE;

  sConfig.Channel = ADC_CHANNEL_14;              /* PA2 = URX */
  sConfig.Rank    = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_15;              /* PA3 = URY */
  sConfig.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* H7 必须先校准，否则读数会偏 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  BSP_Power_MarkReady();
}

/* -------------------------------------------------------------------------- */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
