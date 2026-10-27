# STM32H743 云台视觉系统 — CubeMX 引脚与外设配置

> 配套文件：[PROJECT_RULES.md](../PROJECT_RULES.md) · [pinmap.csv](pinmap.csv)
> 器件：**STM32H743VIT6 / LQFP100**（当前）；ZIT6 / LQFP144 引脚完全兼容，本表通用于两者 · 工具：STM32CubeMX 6.x + HAL
> **舵机驱动：PCA9685 I2C 16 路驱动板**（TIM2 直驱方案转为备选）
> **分阶段**：Phase 1（当前，**无摄像头**）= 舵机 + 串口 + 50Hz 节拍；Phase 2（摄像头到位）= DCMI / SCCB / XCLK / 控制脚，纯增量追加
> 本文所有 AF 编号按 STM32H743 数据手册复用功能表；摄像头 8-bit 映射与实板
> [WeAct MiniSTM32H7xx + OV2640 屏蔽板](https://docs.zephyrproject.org/4.2.1/boards/shields/weact_ov2640_cam_module/doc/) 交叉校验一致。

---

## 0. 前提假设（未指定处以此为准，可逐条推翻）

| # | 假设 | 若不符的影响 |
|---|---|---|
| A1 | 摄像头 = **板载 DCMI 并口 8-bit**（OV5640/OV2640），**Phase 2 才启用**，当前不配置 | 若最终改用「USB 摄像头 + 上位机」，则 15 个摄像头脚永不启用，只保留串口 |
| A2 | 视觉主链路 = **USART1 @921600**（上位机 OpenCV 方案复用同一条口） | 改 USB CDC 见 §7 |
| A3 | 自制板/核心板，**PA13/PA14 = SWD**，HSE 为 8MHz 或 25MHz 无源晶振 | 时钟树参数按实际晶振填（§4） |
| A4 | 不用 SDMMC / USB / ETH / FMC-SDRAM | 本映射刻意避开 PC8–PC12(SD)、PE 口 FMC 常用脚 |
| A5 | 舵机信号 3.3V 电平兼容（PCA9685 板 VCC=3.3V 时输出 3.3V） | 若 VCC=5V 输出 5V 电平，H7 引脚 FT 可承受，但**不可与摄像头共用总线** |
| A6 | **舵机经 PCA9685 驱动板控制**，STM32 只出 I2C | 无驱动板时的直驱方案见 §7 |

---

## 0.5 分阶段启用（**当前只配 Phase 1**）

| 阶段 | 启用内容 | 引脚数 | 前提 |
|---|---|---|---|
| **Phase 1（现在）** | SWD、HSE、I2C2、PD6(OE)、USART1、TIM6、PB0(LED) | 10 | 有 STM32 + PCA9685 + 舵机 |
| **Phase 2（摄像头到货）** | DCMI 11 根 + I2C1(SCCB) 2 根 + XCLK + PWR/RST/PWDN | +15 | 摄像头模组确定后再加 |

> **现在不要在 CubeMX 里勾 DCMI / I2C1 / PA8 / PA7 / PD4 / PD5**：没有器件的复用脚是悬空输入，既无意义，又容易在后续改动时留下"配了但从没用过"的隐患。等模组到货按 §5 的 Phase 2 步骤一次性补齐，改动是纯增量，不会动已经验证过的 Phase 1 部分。

### Phase 1 引脚清单（现在就要配的全部引脚）

| 引脚 | 功能 | 阶段 |
|---|---|---|
| PH0 / PH1 | HSE 晶振 | P1 |
| PA13 / PA14 | SWD 调试 | P1 |
| PB10 / PB11 | I2C2 → PCA9685（SCL/SDA） | P1 |
| PD6 | SERVO_OE（低有效，初始 High） | P1 |
| PA9 / PA10 | USART1 921600（上位机/调试） | P1 |
| PB0 | LED_RUN 心跳 | P1 |
| PA6 / PA4 / PB7 / PC6 / PC7 / PE0 / PE1 / PE4 / PD3 / PE5 / PE6 | DCMI 11 根 | P2 |
| PB8 / PB9 | I2C1 SCCB | P2 |
| PA8 | MCO1 → CAM_XCLK 24MHz | P2 |
| PA7 / PD4 / PD5 | CAM_PWR_EN / PWDN / RST | P2 |

---

## 1. 引脚总表

### 1.1 时钟与调试（不可改）

| 引脚 | CubeMX 信号 | GPIO 模式 | AF | 上下拉 | 输出速度 | Label | 备注 |
|---|---|---|---|---|---|---|---|
| PH0 | RCC_OSC_IN | HSE 晶振输入 | — | 无 | Low | HSE_IN | 8/25MHz，负载电容 18–22pF(8M) / 10pF(25M) |
| PH1 | RCC_OSC_OUT | HSE 晶振输出 | — | 无 | Low | HSE_OUT | 同上 |
| PA13 | SYS_JTMS-SWDIO | 复用推挽 | AF0 | 上拉 | Low | SWDIO | 保持默认，勿复用 |
| PA14 | SYS_JTCK-SWCLK | 复用推挽 | AF0 | 无 | Low | SWCLK | 保持默认，勿复用 |

### 1.2 舵机驱动（PCA9685，I2C2）

| 引脚 | CubeMX 信号 | GPIO 模式 | AF | 上下拉 | 输出速度 | Label | 备注 |
|---|---|---|---|---|---|---|---|
| PB10 | I2C2_SCL | 复用**开漏** | AF4 | 上拉 | Low | SERVO_SCL | 外部 2.2–4.7k 上拉（与板载上拉二选一，别重复） |
| PB11 | I2C2_SDA | 复用**开漏** | AF4 | 上拉 | Low | SERVO_SDA | 同上 |
| PD6 | GPIO_Output | 推挽输出 | — | 无 | Low | SERVO_OE | **初始 High**（OE 低有效，上电先关输出）※可选 |

> ※ 部分 PCA9685 板未引出 OE（板载下拉＝常使能），此时 PD6 不配，PCA9685 上电默认输出 0 脉宽，同样安全。
> **PA0–PA3 从 TIM2 释放**，留作扩展（第三轴、激光笔、补光），当前不配置。

### 1.3 串口（视觉/调试）

| 引脚 | CubeMX 信号 | GPIO 模式 | AF | 上下拉 | 输出速度 | Label | 备注 |
|---|---|---|---|---|---|---|---|
| PA9 | USART1_TX | 复用推挽 | AF7 | 上拉 | Medium | UART_TX | 921600 8N1 |
| PA10 | USART1_RX | 复用推挽 | AF7 | 上拉 | Medium | UART_RX | DMA + IDLE 中断收帧 |

### 1.4 DCMI 摄像头并口（8-bit，硬件同步，全部 AF13）—— **Phase 2，当前不配置**

| 引脚 | CubeMX 信号 | GPIO 模式 | AF | 上下拉 | 输出速度 | Label | 备注 |
|---|---|---|---|---|---|---|---|
| PA6 | DCMI_PIXCLK | 复用推挽 | AF13 | 无 | **Very High** | CAM_PCLK | 像素时钟，边沿敏感 |
| PA4 | DCMI_HSYNC | 复用推挽 | AF13 | 无 | **Very High** | CAM_HSYNC | 行同步 |
| PB7 | DCMI_VSYNC | 复用推挽 | AF13 | 无 | **Very High** | CAM_VSYNC | 场同步 |
| PC6 | DCMI_D0 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D0 | |
| PC7 | DCMI_D1 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D1 | |
| PE0 | DCMI_D2 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D2 | |
| PE1 | DCMI_D3 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D3 | |
| PE4 | DCMI_D4 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D4 | |
| PD3 | DCMI_D5 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D5 | |
| PE5 | DCMI_D6 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D6 | |
| PE6 | DCMI_D7 | 复用推挽 | AF13 | 无 | **Very High** | CAM_D7 | |

> DCMI_D12/D13 仅 12-bit 传感器才需要，本方案 8-bit 悬空不分配。

### 1.5 SCCB（OV 系列寄存器配置，I2C1 —— 与舵机总线分开）—— **Phase 2，当前不配置**

| 引脚 | CubeMX 信号 | GPIO 模式 | AF | 上下拉 | 输出速度 | Label | 备注 |
|---|---|---|---|---|---|---|---|
| PB8 | I2C1_SCL | 复用**开漏** | AF4 | 上拉 | Low | SCCB_SCL | 外部 4.7k **上拉到 3.3V** |
| PB9 | I2C1_SDA | 复用**开漏** | AF4 | 上拉 | Low | SCCB_SDA | 外部 4.7k **上拉到 3.3V** |

> **为什么分成两条 I2C**：PCA9685 板常见 VCC=5V，其板载上拉会把总线拉到 5V；OV5640 的 SCCB 引脚不耐 5V。
> 分开后：I2C2 可 5V 上拉（PB10/PB11 是 FT 引脚），I2C1 保持 3.3V 纯净；也便于单条总线故障隔离。

### 1.6 状态指示（**Phase 1**）

| 引脚 | CubeMX 信号 | GPIO 模式 | AF | 上下拉 | 输出速度 | Label | 初始电平 |
|---|---|---|---|---|---|---|---|
| PB0 | GPIO_Output | 推挽输出 | — | 无 | Low | LED_RUN | Low（心跳灯） |

可选：`PC13` = KEY_USER（GPIO_Input 上拉，注意 3mA 驱动能力）、`PC0` = ADC1_INP10 测舵机母线电压。

### 1.7 摄像头控制（**Phase 2，当前不配置**）

| 引脚 | CubeMX 信号 | GPIO 模式 | AF | 上下拉 | 输出速度 | Label | 初始电平 |
|---|---|---|---|---|---|---|---|
| PA8 | RCC_MCO_1 | 复用推挽 | AF0 | 无 | **Very High** | CAM_XCLK | —（24/25MHz 主时钟） |
| PA7 | GPIO_Output | 推挽输出 | — | 无 | Low | CAM_PWR_EN | **High**（模组上电） |
| PD4 | GPIO_Output | 推挽输出 | — | 无 | Low | CAM_PWDN | **Low**（PWDN 低=工作） |
| PD5 | GPIO_Output | 推挽输出 | — | 无 | Low | CAM_RST | **High**（复位释放） |

### 1.8 电源与复位脚（非 GPIO，但必须接对）

| 引脚 | 连接 |
|---|---|
| VDD（多组） | 3.3V，每对电源脚就近 100nF，另加 4.7µF |
| VDDA | 3.3V 经磁珠/LC 滤波，1µF + 10nF |
| VREF+ | 同 VDDA，1µF + 100nF |
| VBAT | 3V 纽扣电池或直接接 VDD |
| VCAP1 / VCAP2 | **各 2.2µF 到 GND**，必须就近，缺失会随机复位 |
| PDR_ON | 10k 上拉到 VDD（悬空会异常） |
| NRST | 100nF 到 GND |
| BOOT0 | 10k 下拉到 GND |
| VSS | GND |

**PCA9685 板侧**：`VCC` = 3.3V（逻辑，与 MCU 同域，避免 5V 上拉问题）；`V+` = **独立 5–8.4V 大电流电源**，与 MCU **共地**，入口 1000µF 低阻电解 + 0.1µF。

---

## 2. 外设参数

### 2.1 I2C2 + PCA9685（取代 TIM2 直驱）

| 参数 | 值 | 说明 |
|---|---|---|
| I2C2 Mode | I2C | 100kHz，7-bit，开漏 |
| PCA9685 地址 | `0x40`（7-bit） | A0–A5 跳线可改，代码里做成宏 |
| MODE1 (0x00) | `0x11` → 清 SLEEP → `0x01` | 上电默认 sleep，必须先写 prescale 再唤醒 |
| MODE2 (0x01) | `0x04`（OUTDRV=1） | 推挽输出，适合舵机 |
| PRESCALE (0xFE) | **121** | 25MHz/(4096×50)−1；实测 50.03Hz |
| 输出周期 | 19.99ms | 12-bit，1 LSB = **4.88µs** |
| 通道映射 | CH0 = 水平(PAN)，CH1 = 俯仰(TILT) | 其余 14 路空闲 |

**脉宽 → 寄存器：**

```
off_count = round(pulse_us * 4096 / 19989)   // ≈ pulse_us * 0.20491
pulse_us  = 500 + angle_deg / 180 * 2000      // 限幅 [500, 2500]
```

| 角度 | 0° | 45° | 90° | 135° | 180° |
|---|---|---|---|---|---|
| 脉宽 µs | 500 | 1000 | 1500 | 2000 | 2500 |
| off_count | 102 | 205 | 307 | 410 | 512 |

> **分辨率损失**：12-bit @50Hz → 0.44°/LSB，比 TIM2 直驱（0.09°）粗 5 倍。
> 缓解：PID 输出做 1 LSB 死区 + **相邻帧 ±1 LSB 抖动**（Σ-Δ 式），平均分辨率可回到 ~0.2°。

### 2.2 USART1

| 参数 | 值 |
|---|---|
| Mode | Asynchronous |
| Baud | 921600，8 data bits，None parity，1 stop |
| DMA | USART1_RX → **Circular**（CubeMX 自动分配 DMAMUX 通道） |
| 中断 | USART1 global（处理 IDLE 帧间隔） |
| TX | IT 方式发送 |

### 2.3 I2C1（SCCB）—— **Phase 2**

| 参数 | 值 |
|---|---|
| Mode | I2C，Standard 100kHz，7-bit |
| 上拉 | 外部 4.7k 到 **3.3V** |

### 2.4 DCMI —— **Phase 2**

| 参数 | 值 |
|---|---|
| Capture mode | Continuous |
| Data width | **8 bits**（D0–D7） |
| Synchronization | **Hardware**（HSYNC/VSYNC） |
| HSYNC/VSYNC polarity | **High / High** |
| PIXCLK polarity | **Rising** |
| JPEG | 关闭（RGB565/YUV422） |
| DMA | DCMI DMA Settings → Add，Circular + 双缓冲 |
| 中断 | DCMI global（帧结束）+ DMA |

### 2.5 XCLK（PA8 / MCO1）—— **Phase 2**

- 模组自带 24MHz 有源晶振 → PA8 可不配。
- 需要外部 XCLK → **MCO1 = PLL1Q，/5**：`120MHz / 5 = 24MHz`。
- HSE=25MHz 且模组允许 25MHz → `MCO1 = HSE，/1`。

### 2.6 TIM6（50Hz 控制节拍，不占引脚）

| 参数 | 值 |
|---|---|
| Prescaler | 239（240MHz → 1MHz） |
| Counter Period | 19999（20ms / 50Hz） |
| 中断 | TIM6 global，NVIC 抢占 1 |
| 用途 | PID 计算、PCA9685 刷新、视觉外推 |

> 用基本定时器 TIM6 做节拍，把 TIM2/TIM5 完整留给直驱备选或后续三轴扩展。

---

## 3. DMA / NVIC / MPU

### 3.1 NVIC 优先级（Priority Group 4，4 bit 抢占）

| 中断 | 抢占优先级 | 说明 |
|---|---|---|
| TIM6 | 1 | 50Hz 控制节拍 |
| DCMI | 2 | 帧结束处理 |
| USART1 / DMA | 3 | 数据通路 |
| I2C2 event / error | 4 | PCA9685 写入 |
| SysTick | 15 | 默认 |

0 级留给 HardFault/BusFault/NMI。

### 3.2 MPU（H7 必配，否则 D-Cache 会读到旧数据）

| Region | 基址 | 大小 | 属性 | 用途 |
|---|---|---|---|---|
| R0 | `0x2400_0000` | 256KB | Normal，**Non-Cacheable，Shareable**（TEX=1,C=0,B=1,S=1） | DCMI 帧缓冲、UART DMA 环形缓冲 |
| R1 | `0x2404_0000` | 256KB | 默认 Cacheable Write-back | 算法中间结果、大数组 |

- CORTEX_M7 同时开启 **I-Cache + D-Cache**。
- DTCM（`0x2000_0000`，128KB）不进 cache、DMA 不可达：放中断栈、PID 状态、协议解析状态机。

---

## 4. 时钟树（480MHz）

| 域 | 目标频率 | 来源 / 分频 |
|---|---|---|
| SYSCLK / CPU | **480 MHz** | PLL1P |
| AXI / HCLK | 240 MHz | HPRE = /2 |
| APB1/2/3 (PCLK) | 120 MHz | D2PPRE1/2、D1PPRE = /2 |
| APB4 | 120 MHz | D3PPRE = /2 |
| TIM2/3/4/5/6 内核 | 240 MHz | APB1 定时器时钟 ×2 |
| PLL1 VCO | 960 MHz | 480MHz 标准解（PLLP=/2） |
| PLL1Q | 120 MHz | → MCO1 /5 = 24MHz XCLK |
| VOS | **VOS0** | 480MHz 必须 |
| Flash latency | CubeMX 自动（VOS0 约 4WS） | |

**HSE = 8MHz**：PLLM=4 → PLLN=480 → VCO 960 → PLLP=2 → 480MHz；PLLQ=8 → 120MHz。
**HSE = 25MHz**：PLLM=5 → PLLN=192 → VCO 960 → PLLP=2 → 480MHz；PLLQ=8 → 120MHz。

> **⚠️ 固件实际默认用 HSI（免晶振）**：`HSI 64MHz → PLLM=16 → 4MHz → PLLN=240 → VCO 960MHz → PLLP=2 → 480MHz`。
> 注意 PLLN 是 **240**（4MHz × 240 = 960MHz）。写成 480 会让 VCO 跑到 1920MHz，PLL 锁不住、卡进 `Error_Handler`（这是实际踩过的坑）。
> 这样任何 H743 板子（有无晶振、晶振频率未知）都能直接启动。
> 改用外部晶振只改 `firmware/Core/Src/main.c` 三处：`OscillatorType`→HSE、`PLL.PLLSource`→HSE、`PLL.PLLM`（25MHz→5，8MHz→4），PLLN 相应改为 192 / 480。

---

## 5. CubeMX 操作顺序

### Phase 1（现在，10 个引脚）

1. **File → New Project** → `STM32H743ZITx` → LQFP144。
2. **System Core → RCC**：HSE = `Crystal/Ceramic Resonator`；LSE = Disable。
3. **Clock Configuration**：按 §4 填 M/N/P/Q，System Clock Mux = PLL1，HCLK 480，APB 自动 120。
4. **System Core → SYS**：Debug = `Serial Wire`；Timebase = SysTick。
5. **System Core → CORTEX_M7**：ICache/DCache = Enable；MPU 加 R0/R1。
6. **System Core → NVIC**：Priority Group = 4 bits。
7. **Timers → TIM6**：Activated，PSC=239 / ARR=19999，勾选 TIM6 global interrupt。
8. **Connectivity → I2C2**：I2C，100kHz。
9. **Connectivity → USART1**：Asynchronous 921600 8N1 → DMA Add `USART1_RX` = Circular → NVIC 勾选 USART1 global。
10. **Pinout 视图**：PB10/PB11 = I2C2 自动分配；PD6、PB0 设为 GPIO_Output。
11. **GPIO 面板**：PD6 初始电平 **High**（OE 关闭），PB0 初始 Low；速度均 Low，无上下拉。
12. **右键 Enter User Label**：`SERVO_SCL` / `SERVO_SDA` / `SERVO_OE` / `LED_RUN`。
13. **Project Manager**：Toolchain 选 MDK-ARM / STM32CubeIDE；勾选 `Generate peripheral initialization as a pair of .c/.h files per peripheral`。
14. **GENERATE CODE** → 编译 0 error 再写业务代码。

### Phase 2（摄像头到货后追加，**不改动 Phase 1 已验证的配置**）

15. **Multimedia → DCMI**：按 §2.4 设置并 Add DMA。
16. **Connectivity → I2C1**：100kHz（SCCB）。
17. **Pinout**：PA6/PA4/PB7/PC6/PC7/PE0/PE1/PE4/PD3/PE5/PE6 = DCMI；PB8/PB9 = I2C1；PA8 = MCO1；PA7/PD4/PD5 = GPIO_Output。
18. **GPIO 电平**：PA7=High、PD4=Low、PD5=High。
19. 重新 **GENERATE CODE**（CubeMX 在 `USER CODE` 区内合并，不覆盖手写代码）→ 编译。

---

## 6. 生成代码对照（Label → HAL 宏）

| Label | 生成宏 | 使用示例 |
|---|---|---|
| SERVO_SCL/SDA | `hi2c2` | `HAL_I2C_Mem_Write(&hi2c2, 0x40<<1, reg, 1, &val, 1, 10)` |
| SERVO_OE | `SERVO_OE_Pin` / `SERVO_OE_GPIO_Port` | 初始化完成后 `HAL_GPIO_WritePin(..., GPIO_PIN_RESET)` 使能 |
| CAM_PWR_EN / CAM_RST / CAM_PWDN | 同上 | 上电时序：PWR_EN↑ → 10ms → RST↓ → 1ms → RST↑ → 20ms 后读 ID |
| UART_TX/RX | `huart1` | `HAL_UARTEx_ReceiveToIdle_DMA(&huart1, buf, len)` |
| 控制节拍 | `htim6` | `HAL_TIM_PeriodElapsedCallback()` 里跑 PID |

> 业务层只调 `bsp_servo_*` / `bsp_cam_*` / `app_*`，不直接碰 `MX_*`。

---

## 7. 变体与冲突矩阵

| 变体 | 变化 | 说明 |
|---|---|---|
| **无驱动板，TIM2 直驱** | PA0/PA1 = TIM2_CH1/CH2（AF1），PSC=239、ARR=19999、CCR 500–2500 | 分辨率 0.09°，优于 PCA9685；引脚方案回到上一版 |
| **PCA9685 与摄像头共用 I2C1** | 可省两根线 | ❌ 不推荐：5V 上拉会打到 OV5640；总线锁死会同时丢摄像头 |
| **无摄像头（即当前 Phase 1）** | 只配 §0.5 的 10 个脚 | 用串口注入模拟目标坐标，先把 PID 闭环调通 |
| **USB CDC 替代串口** | PA11/PA12 AF10；USART1 挪 PB14/PB15(AF4) | PA9 是 USB_VBUS 检测脚 |
| **用 SD 卡** | SDMMC1：PC8–PC12 + PD2 | 本方案不占用 |
| **板载 SDRAM/ETH** | FMC/ETH 常用 PE0/PE1/PE4/PE5/PE6 | ⚠️ **按实际走线复核这 5 个脚** |

---

## 8. 验收标准

### Phase 1（现在就能验）

1. CubeMX Pinout 视图 **0 个红色冲突、0 个 "!"**；只有 §0.5 那 10 个脚被占用。
2. `HAL_RCC_GetSysClockFreq()` = 480MHz，HCLK 240MHz，PCLK1 120MHz。
3. I2C2 扫描到 `0x40`；读回 PRESCALE = **121**；MODE1 的 SLEEP 位已清零。
4. 示波器测 CH0：**1.5ms 高 / 19.99ms 周期**（50.03Hz）；写 102/512 对应 0.5/2.5ms。
5. PD6 上电为高（输出关闭），初始化完成后才拉低；期间舵机不抖。
6. 串口 921600 回环 0 误码。
7. **闭环自测**：PC 端按协议注入"模拟目标坐标"，舵机跟随误差 ≤ 1 LSB（0.44°），无持续振荡。

### Phase 2（摄像头到货后）

8. DCMI 出图，RGB565 单帧 CRC 连续 100 帧稳定。
9. 读 OV 芯片 ID = 0x5640 / 0x2640，能写入曝光/白平衡并生效。
10. 真实目标接入后，像素误差 → 角度 → 舵机整链闭环收敛，超调 < 10%。

---

## 9. 风险与 PCB 提示

- **V+ 载流**：双 25KG 堵转合计 6–10A，PCA9685 板 V+ 端子与走线通常只扛 3–5A。**禁止让舵机大电流穿过驱动板**：舵机红/黑线直接接独立电源汇流排，只把信号线与 GND 接到驱动板；否则轻则掉压抖动，重则烧板。
- **上拉电压**：PCA9685 板 VCC 若接 5V，其板载上拉是 5V —— 只允许出现在 I2C2 上（PB10/PB11 为 5V 容忍的 FT 脚），绝不能与 SCCB 共线。
- **分辨率**：0.44°/LSB 会限制最小可分辨误差；PID 死区 ≥1 LSB + 帧间抖动，否则出现 ±1 级极限环。
- **内部振荡器**：PCA9685 用 25MHz RC，频率误差几个百分点，表现为 50Hz 轻微漂移，舵机不敏感；对时间敏感的应用需外供时钟。
- **总线锁死**：上拉 2.2–4.7k；SDA 被从机拉死时用 I2C 重新初始化（`HAL_I2C_DeInit/Init`）+ 9 个 SCL 脉冲恢复。
- 摄像头排线 **>10cm** 时把 PIXCLK 降到 12–24MHz，否则采样错位。
- PA13/PA14 上不要挂器件；上电顺序先 3.3V 再舵机电源，断电反向。
