# 二自由度云台视觉跟踪系统 — 项目规则

> 目标平台：**STM32H743**（Cortex-M7 @480MHz）· 当前器件：**STM32H743VIT6 / LQFP100**（原 ZIT6/LQFP144，引脚完全兼容）
> 本文件为项目最高优先级约定，**每次执行任务前必须先读取**；与用户当次指令冲突时，以当次指令为准，并回写本文件。

---

## 1. 硬件与电源（硬约束，不可违反）

| 项 | 规格 |
|---|---|
| 主控 | STM32H743VIT6（LQFP100；与原 ZIT6/LQFP144 引脚兼容），Cortex-M7 480MHz，2MB Flash / 1MB RAM |
| 执行机构 | 2-DOF 云台，2 × XTARK 25KG 大扭矩金属数字舵机 |
| 舵机信号 | 50Hz（20ms），脉宽 0.5–2.5ms（对应约 0–180°），3.3V/5V 逻辑兼容 |
| 舵机供电 | **独立 5–8.4V 大电流电源**（建议 6V/10A 级 BEC，或 2S 锂电 + UBEC） |
| 接地 | 舵机电源 GND **必须**与 STM32 开发板 GND 共地（单点星型接地） |

**禁止项：**
- ❌ 严禁用开发板 3.3V/5V 引脚给 25KG 舵机供电（堵转电流可达数安培，会拉垮 MCU / 烧稳压器）。
- ❌ 严禁用软件延时翻转 GPIO 产生 PWM。
- ❌ 严禁舵机地与 MCU 地不共地。

**工程建议：** 舵机电源入口并联 1000µF 低阻电解 + 0.1µF 陶瓷；信号线可串 100–330Ω 并在舵机端对地 10k 下拉；两轴分别走线，避免共阻抗耦合。

---

## 2. 舵机驱动与 PWM

- **主方案：PCA9685 I2C 驱动板**（I2C2 = PB10/PB11 @100kHz，7-bit 地址 0x40），STM32 只下发 12-bit 占空比，TIM2 不占引脚。
- PCA9685：`PRESCALE = 121` → 50.03Hz；`off_count = round(pulse_us * 4096 / 19989)` ∈ [102, 512]；1 LSB = 4.88µs ≈ **0.44°**。
- 12-bit 分辨率不足时：PID 输出加 1 LSB 死区 + 相邻帧 ±1 LSB 抖动（Σ-Δ），平均分辨率可回到 ~0.2°。
- **备选（无驱动板）：TIM2 / TIM5 直驱**，`PSC = 239`、`ARR = 19999`，`CCR = pulse_us ∈ [500, 2500]`，分辨率 0.09°。
- 角度 → 脉宽：`pulse_us = 500 + angle_deg / 180.0f * 2000.0f`，限幅 `[500, 2500]`。
- 封装为 `bsp_servo.c/.h`（PCA9685）或 `bsp_pwm.c/.h`（直驱）；定时器/通道/寄存器/脉宽范围不得硬编码进业务层。
- **安全**：OE（PD6）上电默认拉高关闭输出，初始化完成后再使能；舵机大电流**不得**穿过驱动板 V+，红黑线直连独立电源，仅信号与 GND 过板。

---

## 3. 控制算法（视觉跟踪）

链路：**视觉像素误差 → 角度误差 → PID → 目标角度 → PWM CCR**

- PID 要求：积分限幅（anti-windup）、积分分离、微分项一阶低通滤波、输出限幅、误差死区；提供 `PID_Init / PID_Reset / PID_Update` 完整封装，浮点实现（H7 带 FPU，开 `-mfpu=fpv5-d16 -mfloat-abi=hard`）。
- 控制周期固定 50Hz，与 PWM 定时器更新中断（或独立 TIM 中断）同步；PID 计算放中断回调，禁止浮点长耗时运算进高优先级上下文之外的阻塞路径。
- 视觉帧率低于 50Hz 时，做目标位置保持 + 速度外推（线性预测），避免阶梯抖动。
- 增益按"像素→角度"归一化（`Kp_deg = Kp_px * deg_per_px`），换镜头/分辨率只需改标定系数。

---

## 4. 视觉与通信

**最终选型（已确认）**：**HBVCAM-4M2214HD-2 双目 USB 摄像头接 PC**，PC 端 OpenCV 检测目标，
通过 UART 把像素误差发给 STM32。DCMI 板载方案作废（不占用那 15 个引脚）。

- PC 端：[tools/vision_tracker.py](tools/vision_tracker.py)（OpenCV + pyserial）
- STM32 端：`vision_link.c` 收帧 → `track.c` 的 PID → PCA9685

（摄像头已在手，Phase 1 的虚拟目标发生器保留作为无相机时的自检手段。）

串口协议规范：
- 二进制、小端；帧格式 `0xAA 0x55 | len | cmd | payload | CRC16`。
- 解析必须用**状态机 + 环形缓冲**（DMA + IDLE 中断），严禁阻塞式 `HAL_UART_Receive` 死等。
- CRC16-CCITT 校验失败丢帧并计数；超时（>100ms 无新帧）进入"目标丢失"保护（保持/回中）。

---

## 5. H7 内存映射与缓存（必须考虑）

- 地址空间：DTCM `0x2000_0000`（128KB，**仅 CPU 可访问，DMA1/DMA2 不可达**）、ITCM `0x0000_0000`、AXI SRAM `0x2400_0000`（512KB）、SRAM1-3 `0x3000_0000`、SRAM4 `0x3800_0000`（D3 域，低功耗域）。
- **DMA 缓冲区**必须放在 DMA 可达区域（AXI SRAM / SRAM1-3 / SRAM4），并用 MPU 配置为 **Non-Cacheable**，或传输前后调用 `SCB_CleanDCache_by_Addr` / `SCB_InvalidateDCache_by_Addr`。
- 所有 DMA 描述符、双缓冲、帧缓冲：`__attribute__((aligned(32)))`，长度按 32 字节对齐。
- 中断栈、PID 状态、通信环形缓冲放 DTCM（`.dtcm` 段，最快且不受 cache 影响）；启动文件栈大小需按实际调用深度调整。
- 链接脚本预留 `.itcm` 段，热路径（PID、PWM 更新、协议解析）可放 ITCM 提性能。

---

## 6. 代码风格

- 全 HAL 库，模块化、高内聚低耦合：`bsp_pwm` / `pid` / `gimbal` / `vision_proto` / `app`。
- 关键代码（PID 计算、定时器中断回调、串口解析）必须给出**完整可直接编译**的 `.c` + `.h`。
- 命名：模块前缀（如 `Gimbal_SetAngle`、`PID_Update`），内部函数 `static`，禁止魔法数字。
- 修改前说明影响范围；提供验证方法（示波器测占空比、串口打印波形/曲线）。

---

## 7. 配置基线（已按 docs/CubeMX_PinConfig.md 固化）

- **摄像头**：板载 DCMI 并口 8-bit（OV5640/OV2640），模组自带 24MHz 晶振；无晶振时 PA8/MCO1 供 24MHz。**当前无摄像头 → 属 Phase 2，CubeMX 中先不配这 15 个脚**
- **视觉链路**：USART1 @921600（上位机 OpenCV 方案复用同一口）
- **舵机**：PCA9685（I2C2 = PB10/PB11，地址 0x40，PRESCALE=121）+ OE(PD6)；TIM2 直驱仅作备选
- **控制节拍**：TIM6 50Hz（PSC=239 / ARR=19999），不占引脚
- **SCCB**：I2C1 PB8/PB9 @100kHz，与舵机总线分开（避免 5V 上拉打到摄像头）
- **时钟**：HSI 64MHz → PLL → 480MHz（默认免晶振，可改 HSE），定时器内核 240MHz，VOS0
- **缓存**：D-Cache 开，AXI SRAM 前 256KB 配 Non-Cacheable 作 DMA 缓冲
- **裸机 + 中断**，暂不引入 FreeRTOS

仍需确认（不阻塞开发）：

- 云台机械零位、舵机安装方向、软/硬限位角度
- 板载外设是否占用 PE0/PE1/PE4/PE5/PE6（SDRAM/ETH 常用脚）
- 视觉目标类型（颜色块 / 人脸 / 自定义模型）
