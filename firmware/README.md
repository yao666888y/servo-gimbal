# Phase 1 固件（STM32H743VIT6 + PCA9685）

> 无摄像头阶段的完整固件：480MHz 时钟树、PCA9685 云台驱动、串口协议、50Hz 控制节拍。
> 摄像头相关（DCMI / SCCB / XCLK）留到 Phase 2，见 [docs/CubeMX_PinConfig.md](../docs/CubeMX_PinConfig.md)。

## 1. 已完成状态

| 项 | 状态 |
|---|---|
| 编译 | ✅ 通过（arm-none-eabi-gcc 13.3.1，0 warning） |
| 产物 | `build/gimbal.elf` / `.hex` / `.bin`，ROM **20,200 B (0.96%)**，RAM **9,656 B (7.37%)** |
| 运行验证 | ⏳ 需要你在硬件上跑（本机没有板子） |

## 2. 目录结构

```
firmware/
├── build.ps1                  # 一键构建（推荐）
├── CMakeLists.txt             # 可选：CubeIDE / CLion 用
├── cmake/gcc-arm-none-eabi.cmake
├── STM32H743ZITx_FLASH_DTCMRAM.ld   # DTCM 128K 作 RAM，栈 8K
├── gimbal.ioc                 # 参考用配置（见 §7 说明）
├── Core/
│   ├── Inc/  main.h  app.h  bsp_servo.h  stm32h7xx_it.h  stm32h7xx_hal_conf.h
│   ├── Src/  main.c  app.c  bsp_servo.c  stm32h7xx_it.c  stm32h7xx_hal_msp.c
│   │         system_stm32h7xx.c  syscalls.c  sysmem.c
│   └── Startup/startup_stm32h743zitx.s
└── Drivers/  STM32H7xx_HAL_Driver + CMSIS（来自 STM32Cube FW_H7 V1.12.1）
```

## 3. 构建

```powershell
cd G:\servo-gimbal\firmware
.\build.ps1            # 约 15 秒
.\build.ps1 -Clean     # 全清重编
```

脚本会自动找 `arm-none-eabi-gcc`（本机在 `D:\CLT\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin`）。

> 也可用 CMake：`cmake -S . -B build-cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake`

## 4. 烧录

用 **STM32CubeProgrammer** 或 ST-Link Utility 烧 `build/gimbal.hex`（或 `.bin`，起始地址 `0x08000000`）。
SWD：PA13=SWDIO，PA14=SWCLK，GND，3.3V。

## 5. 上电行为

1. PD6（OE）保持**高** → 舵机无输出、不动；
2. 初始化 I2C2 → 探测 `0x40` → 写 PRESCALE=121 → 唤醒 → MODE2=0x04；
3. 两轴先写 **1500µs 中位**；
4. 拉低 OE 使能，舵机静止在中位；
5. PB0 心跳灯 1Hz 闪烁；串口**默认静默**，收到命令才回一帧（不刷屏）。

若 I2C 没接好，OE 一直为高（舵机不动），心跳灯仍闪 —— 用串口 PING 可进一步确认 MCU 活着。

## 6. 串口协议（USART1，921600 8N1）

```
AA 55 | LEN | CMD | PAYLOAD[LEN-1] | CRC_L CRC_H
LEN   = 1 + payload 长度
CRC16 = CCITT-FALSE (poly 0x1021, init 0xFFFF)，覆盖 LEN、CMD、PAYLOAD
```

| CMD | 名称 | 下行负载 | 应答 |
|---|---|---|---|
| `0x01` | SET_ANGLE | `int16 pan×100`, `int16 tilt×100`（-9000..+9000 = ±90.00°） | 回一帧 `0x84` |
| `0x02` | SET_PULSE | `uint16 pan_us`, `uint16 tilt_us`（500..2500） | 回一帧 `0x84` |
| `0x03` | PING | 无 | `0x83` + 版本 2B |
| `0x04` | GET_STATUS | 无 | `0x84` + 脉宽 4B + V+ 电压 2B(mV) |
| `0x06` | SET_ALL（调试） | `uint16 pulse_us` | 同脉宽写全部 16 路，回 `0x84` |
| `0x07` | READ_REG（调试） | `uint8 reg` | `0x87` + reg + 值（`0xEE` = 读失败） |

> `0x84` 的 6 字节负载：`pan_us(2) tilt_us(2) vbus_mv(2)`，LEN = 7。
> 例：7.40V 中位时 `AA 55 07 84 DC 05 DC 05 2C 07 B4 2F`。

**现成测试帧**（可直接用串口助手发 HEX）：

| 动作 | 帧 |
|---|---|
| PING | `AA 55 01 03 5D 1E` |
| 两轴回中 1500µs | `AA 55 05 02 DC 05 DC 05 BC 05` |
| Pan +30.00°，Tilt −10.00° | `AA 55 05 01 B8 0B 18 FC EA C7` |

## 7. 本次"配置"的落地方式（重要）

- **引脚/外设配置以 `Core/` 下的代码为准**：`main.c`（时钟、GPIO、I2C2、USART1、TIM6）、`stm32h7xx_hal_msp.c`（复用与 NVIC）、`bsp_servo.c`（PCA9685 时序）。
- `gimbal.ioc` 是**按 CubeMX 6.15 格式手写的参考配置**。本机 CubeMX 无法在无桌面环境运行，因此**没有在 GUI 里验证过**；它的作用是让你以后打开 CubeMX 时能看到引脚布局，**不要**用它直接重新生成去覆盖现有工程。
- 时钟用 **HSI 64MHz → PLL → 480MHz**，不依赖外部晶振，任何 H743 板子都能直接启动。改用晶振只改 `main.c` 里 3 处（`RCC_OSCILLATORTYPE_HSE` / `PLL.PLLSource` / `PLL.PLLM`：25MHz→5，8MHz→4）。

## 8. 上板自测顺序

1. **只上 3.3V**，串口发 PING → 应回 `AA 55 03 83 00 01 xx xx`（版本 0.1）。
2. 接 PCA9685（VCC/GND/SCL/SDA），再发 PING → 同上；发 `0x84` 应回两轴 1500。
3. 上舵机电源（**先不接舵机**），量排针 G–V+ 电压。
4. 接一个舵机，发回中帧 → 舵机应缓慢停在中位，不"啪"一下打极限。
5. 发 `SET_PULSE` 1000 / 2000 → 观察平滑转动与示波器 1.0ms / 2.0ms。

> 详细接线见 [docs/Wiring_PCA9685.md](../docs/Wiring_PCA9685.md)。

## 7.5 摇杆手动控制（当前运行模式）
舵机**保持接在 PCA9685 上不动**，只把摇杆接到 H7：

| 摇杆模块 | 接到 | 说明 |
|---|---|---|
| VCC | 3.3V | 几 mA，可以带 |
| GND | GND | |
| **URX** | **PA2** | ADC1_INP14 |
| **URY** | **PA3** | ADC1_INP15 |
| **SW** | **PB1** | 内部上拉，不用外接电阻 |

控制链路：`ADC(DMA) → -100~100 → 死区±10 → 一阶滤波 → 角度0~180 → 脉宽 → PCA9685`

参数都在 [Core/Inc/joystick.h](Core/Inc/joystick.h)：`JOY_DEADBAND`、`JOY_X_INVERT`、`JOY_Y_INVERT`、`SERVO_DRV_PCA9685`。

两个保护：

1. **上电保护**：ADC 值不在 `[500, 3600]` 内视为摇杆未接，锁中位 1500µs，避免悬空被当满行程、舵机撞限位
2. **限位**：角度硬限制 0~180°，对应脉宽 500~2500µs

切换输出方式：`joystick.h` 里 `SERVO_DRV_PCA9685`
- `1`（默认）= PCA9685（I2C2），舵机接线不用动- `0` = TIM2 直驱，舵机信号要改接 PA0/PA1

## 8.5 已踩过的坑（重要）

| 现象 | 根因 | 修复 |
|---|---|---|
| 灯不闪、串口无输出 | PLLN 写成 480，VCO 跑到 1920MHz 锁不住 → `Error_Handler` | PLLN = **240** |
| I2C 全 ACK、寄存器能读，但 **S 引脚 0V 无 PWM** | MODE1 的 **AI 位**（bit5）没置位，4 字节写 LED 寄存器只写第一个 → OFF 恒为 0 | MODE1 写 **0x31**（睡眠时）/ **0x21**（唤醒后） |
| 舵机插上不动作、手拧很松 | 上述 AI 位问题；另注意舵机插头针序 | 先量 S–G ≈ 0.25V 再查舵机 |
| 芯片连不上 / 不执行 | 板子只靠 ST-Link 3.3V 供电，带不动 H743 | 板子必须用 USB/DC 独立供电 |

## 8.6 PID 跟踪（虚拟目标，等待摄像头）

现在固件里带了一个**虚拟目标发生器**，不需要摄像头就能验证整条闭环：

- **短按**摇杆帽 = 立即回中
- **长按 800ms** = 切换 摇杆手动 ⇄ 虚拟目标跟踪
- 跟踪模式下，虚拟目标在 640×480 画面里做 0.15Hz / 0.11Hz 正弦运动，
  固件把它投影成"像素误差"，PID 驱动云台去追

实测（SWD 读 `g_pan_deg`）：`101.9 → 117.2 → 111.2 → 101.7°`，平滑跟随无振荡。

| 文件 | 内容 |
|---|---|
| [pid.c](Core/Src/pid.c) | 增量式 PID：微分低通、误差死区、输出限幅（限幅即抗积分饱和） |
| [track.c](Core/Src/track.c) | 虚拟目标 + 模拟摄像机 + 角度限位 + 舵机输出 |

**摄像头接入点只有一个**（track.c）：

```c
ex = (float)(target_px - FRAME_W/2);   // 替换虚拟目标那一段
ey = (float)(target_py - FRAME_H/2);
```

PID、限位、脉宽换算、舵机输出全部复用，一行不用改。

## 9. 还没做（Phase 1 之后）

- 视觉误差 → PID 闭环（当前是"目标脉宽直接跟随 + 斜率限幅"）
- USART 改 DMA + IDLE 接收（现在逐字节中断，921600 下够用，但协议量大时换）
- 摄像头 DCMI / SCCB / XCLK（Phase 2）
- 亚 LSB 抖动（PCA9685 12-bit 0.44°/级）
