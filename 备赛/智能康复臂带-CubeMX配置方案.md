# AI-RehabBand 智能康复臂带 CubeMX 配置方案

本文档基于 `智能康复臂带-MVP方案.md`，面向 STM32U575RIT6 单节点版本，用于在 STM32CubeMX 中完成工程初始化、外设分配、中断/DMA、时钟和基础工程选项配置。

## 1. 配置目标

MVP 阶段需要在 STM32U575RIT6 上同时完成：

- 单个 IMU660RB 采集：每块 STM32U575RIT6 只连接 1 个陀螺仪/IMU
- 1 路简化肌电 ADC 采集
- P169H002 LCD 触摸显示屏显示刷新（LCD 驱动：ST7789T3，触摸：CST816D）
- 无线数据串口模块接口（USART1，HM-10 / ESP32-C3 / 其他 3.3V TTL 串口透传模块）
- 无线调试串口接口（USART2，独立打印日志和调试命令）
- PA12 单按键预留，当前不参与业务逻辑
- 蜂鸣器或 LED 异常提醒
- 50Hz-100Hz 运动处理主循环
- 200Hz-500Hz 肌电采样

推荐 CubeMX 配置策略：

- 单个 IMU 使用 SPI1，CS 采用 GPIO 软件片选，IMU 数据读取使用 SPI1 RX/TX DMA；第二块单片机连接另一颗 IMU 时复用同一套 SPI1 配置。
- P169H002 的 LCD 显示单独使用一路 SPI，触摸 CST816D 使用 I2C，避免显示刷新影响 IMU 采样。
- 肌电使用 ADC + DMA 循环采样。
- 无线数据串口使用 USART1，调试无线串口使用 USART2，接收建议中断或 DMA。
- 使用 TIM 定时触发采样节拍，保证算法周期稳定。

## 2. MCU 与工程基础设置

### 2.1 MCU 选择

在 CubeMX 中从 MCU Selector 选择具体芯片型号：

```text
STM32U575RIT6
```

不要按 NUCLEO-U575ZI-Q 或 STM32U575ZITx 建工程。`RIT6` 是 LQFP64 封装，引脚数量少于 ZI 系列，SPI、USART、ADC 和调试口的引脚复用需要按 LQFP64 实际可用引脚重新分配。

### 2.2 Project Manager

推荐配置：

```text
Project Name: AI_RehabBand_MVP
Toolchain / IDE: MDK-ARM 或 STM32CubeIDE
Firmware Package: STM32Cube FW_U5 最新稳定版本
Application Structure: Basic
```

Code Generator 建议：

```text
Generate peripheral initialization as a pair of '.c/.h' files per peripheral: Enable
Keep User Code when re-generating: Enable
Delete previously generated files when not re-generated: Disable
Set all free pins as analog: Enable
```

## 3. 系统时钟配置

### 3.1 RCC

若开发板带外部高速晶振：

```text
HSE: Crystal/Ceramic Resonator
LSE: Crystal/Ceramic Resonator
```

若不需要 RTC，LSE 可不启用。

若不确定板载晶振情况，先使用内部时钟：

```text
HSE: Disable
SYSCLK Source: MSI 或 HSI16
```

### 3.2 Clock Configuration

推荐主频：

```text
SYSCLK: 160 MHz
HCLK: 160 MHz
APB1: 160 MHz
APB2: 160 MHz
APB3: 160 MHz
```

理由：

- 160MHz 对姿态解算、显示刷新和串口通信足够。
- STM32U575RIT6 性能余量较大，后续可加入轻量模型。
- 若使用电池供电并追求续航，调试稳定后可降到 80MHz。

电源配置：

```text
Power Regulator Voltage Scale: Scale 1
```

## 4. 引脚与外设总览

以下为推荐分配，实际引脚需根据开发板排针和原理图调整。

| 功能 | CubeMX 外设 | 推荐用途 |
|---|---|---|
| 本节点 IMU660RB | SPI1：PA5/PA6/PA7 + PA4 CS | 本节点 IMU 数据读取 |
| P169H002 LCD 显示 ST7789T3 | SPI3：PC10/PC12 + PA15/PB8/PB9/PA8 | LCD 显示刷新 |
| P169H002 触摸 CST816D | I2C1：PB6/PB7 + PC4/PC5 | 触摸 I2C、复位、中断 |
| 肌电模拟输入 | ADC1_IN1：PC0 | EMG 原始信号采样 |
| 无线数据串口接口 | USART1：PA9/PA10 | 数据上传、板间同步或手机透传 |
| 无线调试串口接口 | USART2：PA2/PA3 | 调试日志、参数下发、printf |
| RTOS 节拍 | FreeRTOS SysTick | 任务调度，SensorTask 10ms 周期 |
| 肌电节拍 | TIM3 TRGO | 500Hz EMG ADC 触发 |
| 按键预留 | GPIO_Input：PA12 | 单按键预留，当前不起作用 |
| LED/蜂鸣器 | GPIO_Output / PWM | 状态和异常提醒 |
| 调试下载 | SYS | SWD |
| 调试日志 | USART2：PA2/PA3 | 独立无线调试串口 |


### 4.1 STM32U575RIT6 推荐具体引脚分配

以下分配按 STM32U575RIT6 LQFP64 封装整理，当前只配置一个 IMU 节点，优先保证本节点 IMU、P169H002 LCD 触摸显示屏、无线串口和肌电采样互不冲突。CubeMX 中按 `Pin Name` 选择即可，硬件原理图上也按这些网络名标注。

| 模块 | 信号 | STM32U575RIT6 引脚 | CubeMX 配置 | 说明 |
|---|---|---|---|---|
| 本节点 IMU660RB | IMU_SCK | PA5 | SPI1_SCK | SPI1，Mode 3 |
| 本节点 IMU660RB | IMU_MISO | PA6 | SPI1_MISO | IMU -> STM32 |
| 本节点 IMU660RB | IMU_MOSI | PA7 | SPI1_MOSI | STM32 -> IMU |
| 本节点 IMU660RB | IMU_CS | PA4 | GPIO_Output | 软件片选，默认高电平，读写时拉低 |
| 本节点 IMU660RB | IMU_INT | PC8 | GPIO_EXTI，可选 | 数据就绪中断，MVP 可不接 |
| P169H002 LCD 显示 | LCD_SCL | PC10 | SPI3_SCK | ST7789T3 SCL |
| P169H002 LCD 显示 | LCD_SDA | PC12 | SPI3_MOSI | ST7789T3 SDA，显示写入 |
| P169H002 LCD 显示 | LCD_SDO | PC11 | SPI3_MISO，可选 | 显示只写时可不接 |
| P169H002 LCD 显示 | LCD_CS | PA15 | GPIO_Output | ST7789T3 CS，低电平选中 |
| P169H002 LCD 显示 | LCD_DC | PB8 | GPIO_Output | ST7789T3 RS/DC，数据/命令选择 |
| P169H002 LCD 显示 | LCD_RST | PB9 | GPIO_Output | ST7789T3 RESET，低电平复位 |
| P169H002 LCD 显示 | LCD_BL | PA8 | GPIO_Output 或 TIM PWM | 背光控制，按驱动板电路决定有效电平 |
| P169H002 触摸 | TP_SCL | PB6 | I2C1_SCL | CST816D CTP_SCL |
| P169H002 触摸 | TP_SDA | PB7 | I2C1_SDA | CST816D CTP_SDA |
| P169H002 触摸 | TP_RST | PC5 | GPIO_Output | CST816D CTP_TRST，低电平复位 |
| P169H002 触摸 | TP_INT | PC4 | GPIO_Input 或 GPIO_EXTI | CST816D CTP_TINT，触摸中断 |
| 肌电模块 | EMG_ADC | PC0 | ADC1_IN1 | 模拟输入，0-3.3V |
| 无线数据串口模块 | WIRELESS_RXD | PA9 | USART1_TX | STM32 发，模块收 |
| 无线数据串口模块 | WIRELESS_TXD | PA10 | USART1_RX | 模块发，STM32 收 |
| 无线数据串口模块 | WIRELESS_EN | PB0 | GPIO_Output，可选 | 模块使能，不需要可不接 |
| 无线数据串口模块 | WIRELESS_STATE | PB1 | GPIO_Input，可选 | 连接状态，不需要可不接 |
| 无线调试串口模块 | DBG_RXD | PA2 | USART2_TX | STM32 调试输出，接模块 RXD |
| 无线调试串口模块 | DBG_TXD | PA3 | USART2_RX | 模块 TXD，STM32 接收调试命令 |
| 按键预留 | KEY_RESERVED | PA12 | GPIO_Input | 上拉输入，仅预留，当前不起作用 |
| 提示输出 | LED_STATUS | PC6 | GPIO_Output | 运行状态指示 |
| 提示输出 | BUZZER | PB2 | GPIO_Output 或 TIM PWM | 异常提醒 |
| 调试下载 | SWDIO | PA13 | SYS_JTMS-SWDIO | 保留 SWD |
| 调试下载 | SWCLK | PA14 | SYS_JTCK-SWCLK | 保留 SWD |
| 复位 | NRST | NRST | Reset | 建议引出到下载口 |

注意事项：

- `PA13`、`PA14` 必须保留给 SWD 下载调试。
- `PA15`、`PB3`、`PB4` 属于 JTAG 相关脚，使用 SWD 调试时可以作为普通 GPIO/外设使用；本方案只占用 `PA15` 做 LCD_CS，不影响 SWD 下载。
- P169H002 LCD只写数据时可以不接 `PC11/SPI3_MISO`，这样 LCD 排针可以做 7 线：`3V3/GND/SCL/SDA/CS/DC/RST/BL`，其中 BL 可直接上拉常亮或接 `PA8` 控制。
- 如果你的 LCD 模块排针标的是 `SCK/SDA/A0/RES/CS/BLK`，对应关系是：`SCK->PC10`，`SDA->PC12`，`A0/DC->PB8`，`RES/RST->PB9`，`CS->PA15`，`BLK->PA8`。
- 无线数据串口最小接口是 `3V3`、`GND`、`PA9 USART1_TX`、`PA10 USART1_RX`；`PB0 EN` 和 `PB1 STATE` 只是预留。
- 无线调试串口最小接口是 `3V3`、`GND`、`PA2 USART2_TX`、`PA3 USART2_RX`，用于 printf 日志和现场调参。
- 肌电 `PC0/ADC1_IN1` 前端必须限幅到 0-3.3V，不能输入负压或超过 VDDA 的电压。

## 5. SYS 与调试接口

在 `System Core > SYS` 中配置：

```text
Debug: Serial Wire
```

不要关闭 SWDIO/SWCLK 对应引脚，否则后续可能无法下载调试。

## 6. GPIO 配置

### 6.1 IMU 软件片选和中断引脚

本节点只配置 1 个 IMU，使用 SPI1。SPI1 使用软件片选，CubeMX 中 `Hardware NSS Signal` 设为 `Disable`，`NSS Signal Type` 设为 `Software`。IMU 至少需要一个 GPIO 作为 CS 引脚：

```text
IMU_CS: GPIO_Output, default High
```

若使用 IMU 的数据就绪中断，可增加：

```text
IMU_INT: GPIO_EXTI
```

MVP 推荐先不用 IMU INT，使用定时器 100Hz 轮询读取，开发更稳。
软件片选控制原则：

```text
空闲状态: CS = High
开始通信: CS = Low
通信完成: CS = High
```

CS 拉低和拉高建议封装成宏或函数，避免不同驱动里重复写错。

### 6.2 显示屏控制引脚

P169H002 / ST7789T3 常用 LCD 控制引脚：

```text
LCD_CS:  GPIO_Output, default High  // 软件片选
LCD_DC:  GPIO_Output, default Low
LCD_RST: GPIO_Output, default High
LCD_BL:  GPIO_Output 或 TIM PWM
```

若背光只需要常亮，`LCD_BL` 配为 GPIO_Output 即可。

### 6.3 按键预留

当前只保留 1 个按键引脚，先不参与模式切换、开始停止或中断逻辑。

```text
KEY_RESERVED: PA12, GPIO_Input, Pull-Up, No EXTI
```

后续如果需要启用按键，再把 PA12 改为 GPIO_EXTI 或在 UiTask 中轮询消抖。

### 6.4 状态 LED / 蜂鸣器

```text
LED_STATUS: GPIO_Output
BUZZER: PB2, GPIO_Output, default High
```

当前 `.ioc` 使用 `PB2` 作为 BUZZER，默认输出高电平。若后续蜂鸣器需要不同提示音，再把 BUZZER 改为合适的 TIM PWM 输出。

## 7. 单 IMU660RB 配置

IMU660RB 核心芯片为 LSM6DSRTR，常见接口为 SPI。当前每块 STM32U575RIT6 只连接 1 颗 IMU，使用 SPI1。另一块单片机连接远端 IMU 时，也按本节复用同样配置。

### 7.1 SPI1 - 本节点 IMU

在 `Connectivity > SPI1` 中配置：

```text
Mode: Full-Duplex Master
Hardware NSS Signal: Disable
NSS Signal Type: Software
Data Size: 8 Bits
First Bit: MSB First
Clock Polarity: High
Clock Phase: 2 Edge
Baud Rate Prescaler: 32 或 64
```

初始 SPI 频率建议控制在 1MHz-5MHz，传感器初始化稳定后可提高。

LSM6DS 系列常用 SPI Mode 3：

```text
CPOL = 1
CPHA = 1
```

即 CubeMX 中 `Clock Polarity: High`，`Clock Phase: 2 Edge`。


### 7.2 SPI1 DMA - IMU 数据读取

IMU 需要 100Hz 稳定读取，建议 SPI1 同时启用 RX DMA 和 TX DMA。读取寄存器时，TX DMA 发送寄存器地址和 dummy 字节，RX DMA 接收返回数据。

在 `Connectivity > SPI1 > DMA Settings` 中添加：

```text
DMA Channel: GPDMA1 Channel 0
DMA Request: SPI1_RX
Mode: Normal
Data Width Peripheral: Byte
Data Width Memory: Byte
Increment Address Peripheral: Disable
Increment Address Memory: Enable
Priority: High
```

再添加：

```text
DMA Channel: GPDMA1 Channel 1
DMA Request: SPI1_TX
Mode: Normal
Data Width Peripheral: Byte
Data Width Memory: Byte
Increment Address Peripheral: Disable
Increment Address Memory: Enable
Priority: High
```

U5 系列的 GPDMA 通道不是固定绑定外设，关键是同一个 DMA 通道里选择正确的 `DMA Request`。本方案固定写法是：`GPDMA1 Channel 0 + SPI1_RX`，`GPDMA1 Channel 1 + SPI1_TX`。

SPI1 DMA 使用原则：

```text
1. IMU_CS 拉低
2. 调用 HAL_SPI_TransmitReceive_DMA()
3. 在 HAL_SPI_TxRxCpltCallback() 中拉高 IMU_CS
4. 置位 imu_dma_done 标志或释放 RTOS 信号量
```

不要在 DMA 传输未完成时拉高 CS，否则 IMU 本次读数可能不完整。
### 7.3 SPI2 说明

当前节点不配置第二颗 IMU，因此 `SPI2` 不作为必配外设。`PB12/PB13/PB14/PB15` 可保留为空闲 GPIO，后续需要扩展传感器时再启用。

### 7.4 IMU 采样参数

这部分不是 CubeMX 配置，而是在驱动初始化中写寄存器：

```text
加速度计量程: +-4g 或 +-8g
陀螺仪量程: +-500 dps 或 +-1000 dps
ODR: 104Hz
采样读取频率: 100Hz
```

MVP 推荐：

```text
IMU ODR: 104Hz
算法周期: 100Hz
```

这样能满足动作识别和角度计算需求。

## 8. 肌电 ADC 配置

肌电模块若输出模拟包络或模拟原始电压，接入 ADC。

### 8.1 ADC1

在 `Analog > ADC1` 中配置：

```text
Mode: Independent mode
Resolution: 12-bit
Data Alignment: Right alignment
Scan Conversion Mode: Disabled
Continuous Conversion Mode: Disabled
External Trigger Conversion Source: TIM3 TRGO / TIM3 TRGO Event / TIM3 Update Event
External Trigger Conversion Edge: Rising Edge
DMA Continuous Requests: Enable
Overrun: Overrun data overwritten
Sampling Time: 47.5 cycles 或更高
```

通道示例：

```text
ADC1_IN1: EMG_ADC
```

具体 INx 根据开发板可用 ADC 引脚调整。

### 8.2 ADC DMA

在 ADC1 的 DMA Settings 中添加：

```text
DMA Request: ADC1
Mode: Circular
Data Width Peripheral: Half Word
Data Width Memory: Half Word
Increment Address Peripheral: Disable
Increment Address Memory: Enable
Priority: High
```

建议 DMA 缓冲区：

```text
emg_adc_buf[64] 或 emg_adc_buf[128]
```

500Hz 采样下，64 点约 128ms 窗口，适合做整流、平均值、RMS 或包络阈值判断。

## 9. 定时器配置

### 9.1 TIM6 - 100Hz 系统算法节拍

用途：

- 触发主循环读取本节点 IMU
- 更新姿态解算
- 更新动作识别
- 更新评分状态

在 `Timers > TIM6` 中配置：

```text
Mode: Activated
Prescaler: 16000 - 1
Counter Period: 100 - 1
Trigger Event Selection: Update Event
```

当 TIM6 时钟为 160MHz 时：

```text
160MHz / 16000 / 100 = 100Hz
```

FreeRTOS 启用后，TIM6 不再作为算法中断节拍使用，可不启用 TIM6 global interrupt。

### 9.2 TIM3 - 500Hz 肌电 ADC 触发

用途：

- 稳定触发 ADC 采样

在 `Timers > TIM3` 中配置：

```text
Mode: Activated
Prescaler: 16000 - 1
Counter Period: 20 - 1
Trigger Event Selection: Update Event
```

当 TIM3 时钟为 160MHz 时：

```text
160MHz / 16000 / 20 = 500Hz
```

TIM3 不需要开中断，主要用于 TRGO / Update Event 触发 ADC。若 ADC 触发源列表里没有精确的 `TIM3 TRGO`，优先找 `TIM3_TRGO`、`TIM3 TRGO Event`、`TIM3 Update Event` 这类名称。

## 10. 显示屏配置

P169H002 的 LCD 显示驱动为 ST7789T3，触摸芯片为 CST816D。LCD 使用 4 线串行 SPI，触摸使用 I2C。

### 10.1 SPI3 - P169H002 LCD 显示 ST7789T3

在 `Connectivity > SPI3` 中配置：

```text
Mode: Full-Duplex Master 或 Transmit Only Master
Hardware NSS Signal: Disable
NSS Signal Type: Software
Data Size: 8 Bits
First Bit: MSB First
Clock Polarity: Low
Clock Phase: 1 Edge
Baud Rate Prescaler: 4 或 8
```

ST7789 常用 SPI Mode 0：

```text
CPOL = 0
CPHA = 0
```

P169H002 LCD 显示刷新推荐使用阻塞 SPI 先打通，后续再改 DMA。

### 10.2 SPI3 DMA 可选

若需要更流畅刷新，在 SPI3 中添加 TX DMA：

```text
DMA Request: SPI3_TX
Mode: Normal
Data Width Peripheral: Byte
Data Width Memory: Byte
Priority: Medium
```

MVP 的 LCD 显示内容以文字、角度、次数和状态为主，不强制使用 DMA。ST7789T3 与 ST7789 系列驱动相近，但 P169H002 的分辨率、偏移量和初始化命令需要按实物资料校准。

### 10.3 P169H002 LCD 接口排针建议

建议在硬件上给 P169H002 预留接口：

```text
LCD_VCC  -> 3V3
LCD_GND  -> GND
LCD_SCL  -> SPI3_SCK
LCD_SDA  -> SPI3_MOSI
LCD_CS   -> GPIO_Output
LCD_DC   -> GPIO_Output
LCD_RST  -> GPIO_Output
LCD_BL   -> GPIO_Output 或 PWM
TP_SCL   -> I2C1_SCL / PB6
TP_SDA   -> I2C1_SDA / PB7
TP_RST   -> GPIO_Output / PC5
TP_INT   -> GPIO_Input 或 GPIO_EXTI / PC4
```

若屏幕模块带 MISO/SDO 引脚，MVP 阶段可以不接；ST7789T3 显示通常只需要写入数据。
### 10.4 CST816D 触摸 I2C 配置

在 `Connectivity > I2C1` 中配置：

```text
I2C Speed Mode: Fast Mode
I2C Speed Frequency: 400kHz
Addressing Mode: 7-bit
```

触摸引脚：

```text
TP_SCL -> PB6 / I2C1_SCL
TP_SDA -> PB7 / I2C1_SDA
TP_RST -> PC5 / GPIO_Output
TP_INT -> PC4 / GPIO_Input，后续需要中断时改 GPIO_EXTI
```

当前阶段如果只想先点亮屏幕，可以先配置 PB6/PB7/PC4/PC5 为预留，不写触摸驱动；但原理图建议把这 4 根线引出。

## 11. 无线串口接口配置

本节点配置两路串口：`USART1` 作为无线数据串口，`USART2` 作为无线调试串口。两路都可以接 HM-10、ESP32-C3 串口透传模块或其他 3.3V TTL 串口模块，CubeMX 中按普通 UART 处理。

### 11.1 USART1 - 无线数据串口

在 `Connectivity > USART1` 中配置：

```text
Mode: Asynchronous
Baud Rate: 115200
Word Length: 8 Bits
Parity: None
Stop Bits: 1
Hardware Flow Control: None
Oversampling: 16 Samples
```

无线数据串口接口排针建议：

```text
WIRELESS_VCC   -> 3V3
WIRELESS_GND   -> GND
WIRELESS_TXD   -> USART_RX   // 模块发，STM32 收
WIRELESS_RXD   -> USART_TX   // 模块收，STM32 发
WIRELESS_EN    -> GPIO_Output，可选
WIRELESS_STATE -> GPIO_Input，可选
```

最小可用接口只需要 4 根线：`3V3`、`GND`、`USART_TX`、`USART_RX`。如果模块没有 EN/STATE 引脚，可以不配置。

注意无线串口模块电平：

- HM-10 多数为 3.3V TTL，可直连 STM32。
- ESP32-C3 也是 3.3V TTL。
- 不要接 5V 串口电平。


### 11.2 USART2 - 无线调试串口

在 `Connectivity > USART2` 中配置：

```text
Mode: Asynchronous
Baud Rate: 115200
Word Length: 8 Bits
Parity: None
Stop Bits: 1
Hardware Flow Control: None
Oversampling: 16 Samples
```

无线调试串口接口排针建议：

```text
DBG_VCC -> 3V3
DBG_GND -> GND
DBG_RXD -> PA2 / USART2_TX   // STM32 发，调试模块收
DBG_TXD -> PA3 / USART2_RX   // 调试模块发，STM32 收
```

用途建议：

```text
USART1: 训练数据、板间数据、手机端通信
USART2: printf 日志、调试命令、阈值参数临时修改
```

调试串口不要发送高频完整 IMU 原始数据，建议只发降频后的摘要，例如 10Hz 的角度、状态和错误码，避免阻塞系统。
### 11.3 UART 接收方式

MVP 推荐：

```text
USART1 global interrupt: Enable
USART2 global interrupt: Enable
```

使用中断接收无线数据串口和调试串口命令，例如：

```text
START
STOP
MODE,0
MODE,1
MODE,2
```

训练结果发送格式建议：

```text
RESULT,mode,action,count,angle_max,score,emg_rate,abnormal_count
```

示例：

```text
RESULT,1,0,12,96,88,74,1
```

## 12. NVIC 中断优先级建议

在 `System Core > NVIC` 中配置。因为本工程启用 FreeRTOS，凡是在中断回调里调用 `osSemaphoreRelease`、`osMessageQueuePut`、`xSemaphoreGiveFromISR`、`xQueueSendFromISR` 等 RTOS API 的中断，抢占优先级数值必须 `>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`。

CubeMX 默认常见配置：

```text
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
```

STM32 中断优先级数字越小，优先级越高。因此能调用 RTOS FromISR API 的中断不要设成 0-4，建议如下：

| 中断 | 抢占优先级 | 用途 | 是否允许调用 RTOS FromISR API |
|---|---:|---|---|
| SPI1 RX DMA | 5 | IMU 数据接收完成 | 是，释放 `imuDmaDoneSem` |
| SPI1 TX DMA | 5 | IMU 发送完成 | 通常不需要，保持同级 |
| ADC1 DMA | 6 | 肌电采样半满/满缓冲 | 是，可释放 `adcHalfCpltSem/adcCpltSem` |
| USART1 Global | 7 | 无线数据串口接收 | 是，可投递命令队列 |
| USART2 Global | 8 | 无线调试串口接收 | 是，可投递调试命令队列 |
| TP_INT / EXTI4 | 9 | CST816D 触摸中断，可选 | 是，可通知触摸任务 |
| IMU_INT / EXTI8 | 9 | IMU 数据就绪中断，可选 | 是，可通知 SensorTask |
| SPI3 DMA TX | 10 | LCD 刷新完成，可选 | 是，但建议只释放显示信号量 |
| TIM17 Global | 15 | HAL timebase | 否，只给 HAL tick 使用 |
| PA12 Key | - | GPIO_Input 预留，不启用 EXTI | 否 |

原则：

- `TIM17 global interrupt` 必须开启，因为 SYS timebase 改为 TIM17 后，HAL tick 依赖它。
- TIM17 优先级设最低，例如抢占优先级 15，避免影响 SPI/ADC/UART。
- SPI1 DMA、ADC DMA、USART1、USART2 后续大概率会在中断里释放信号量或投递队列，所以统一设为 5 或更大的数字。
- 如果某个中断绝对不调用 RTOS API，只置 `volatile` 标志，理论上可以设为 0-4；本项目为了避免后期维护踩坑，不建议这样做。
- 不要在任何中断里做姿态解算、RMS 计算、LCD 刷新或 printf。
## 13. DMA 汇总

建议强制启用：

```text
SPI1_RX -> GPDMA1 Channel 0, DMA Request SPI1_RX, Normal, High Priority
SPI1_TX -> GPDMA1 Channel 1, DMA Request SPI1_TX, Normal, High Priority
ADC1    -> DMA Circular, High Priority
```

可选启用：

```text
SPI3_TX  -> DMA Normal, Medium Priority
USART1_RX -> DMA Circular 或中断接收
USART1_TX -> DMA Normal
USART2_RX -> DMA Circular 或中断接收
USART2_TX -> DMA Normal
```

当前版本建议强制启用 `SPI1_RX`、`SPI1_TX` 和 `ADC1` DMA。USART1/USART2 可以先用中断接收，稳定后再改 DMA。LCD 的 SPI3_TX DMA 可后续优化显示刷新时再启用。
## 14. FreeRTOS 配置建议

当前版本建议启用 FreeRTOS，使用 CMSIS-RTOS v2。可以把 FreeRTOS 理解成：以前所有逻辑都写在 `while(1)` 里排队执行，现在把不同功能拆成多个“任务函数”，由 RTOS 按优先级和周期调度。

本项目同时有 IMU DMA、ADC DMA、LCD 刷新、无线数据串口和无线调试串口。用 RTOS 拆开后，采样、算法、显示和通信之间不容易互相卡住。

### 14.1 CubeMX FreeRTOS 基础配置

在 `Middleware and Software Packs > FREERTOS` 中配置：

```text
Interface: CMSIS_V2
Kernel: FreeRTOS
USE_NEWLIB_REENTRANT: Enable
Heap Usage Scheme: heap_4
TOTAL_HEAP_SIZE: 24KB 起步，后续按实际内存占用调整
MINIMAL_STACK_SIZE: 256 words
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY: 5
TICK_RATE_HZ: 1000
```

在 `System Core > SYS` 中设置：

```text
Timebase Source: TIM17
```

原因：FreeRTOS 使用 SysTick 作为系统节拍，HAL 的 timebase 建议改用 TIM17，避免 HAL_Delay 和 RTOS tick 互相影响。改成 TIM17 后必须在 NVIC 中开启 `TIM17 global interrupt`，优先级设为 15。

`TIM6` 原先的 100Hz 算法节拍不再启用中断，由 `SensorTask` 用 10ms 周期实现。

### 14.2 新手理解：任务是什么

一个任务可以理解成一个独立循环：

```c
void StartSensorTask(void *argument)
{
    for (;;)
    {
        // 做传感器采集
        osDelay(10);
    }
}
```

RTOS 会在多个任务之间切换。优先级高的任务更容易先运行，所以采样任务优先级要高，显示和日志优先级要低。

本项目的优先级原则：

```text
采样 > 算法/肌电处理 > 通信/状态 > 显示/调试日志
```

### 14.3 CubeMX 任务配置表

在 `FREERTOS > Tasks and Queues` 中添加以下任务：

| Task Name | Entry Function | Priority | Stack Size | 周期/触发 | 作用 |
|---|---|---:|---:|---|---|
| SensorTask | StartSensorTask | osPriorityHigh | 512 words | 10ms | 读取 IMU，启动 SPI1 DMA，等待 DMA 完成 |
| AlgoTask | StartAlgoTask | osPriorityAboveNormal | 1024 words | 收到 IMU 数据后触发 | 姿态解算、角度计算、动作识别、评分 |
| EmgTask | StartEmgTask | osPriorityAboveNormal | 512 words | 20ms | 处理 ADC DMA 肌电缓冲区，计算 RMS/包络/阈值 |
| WirelessTask | StartWirelessTask | osPriorityNormal | 512 words | 100ms 或事件触发 | USART1 数据上传、板间数据、手机命令 |
| UiTask | StartUiTask | osPriorityNormal | 384 words | 20ms | LED、蜂鸣器状态；PA12 按键只预留不处理 |
| DisplayTask | StartDisplayTask | osPriorityLow | 768 words | 100ms-200ms | LCD 刷新角度、次数、评分、状态 |
| DebugTask | StartDebugTask | osPriorityLow | 512 words | 100ms-500ms | USART2 输出调试日志、处理调试命令 |

说明：

- `SensorTask` 优先级最高，因为 IMU 100Hz 采样要稳定。
- `DisplayTask` 和 `DebugTask` 优先级最低，因为屏幕和日志慢一点没关系，不能影响采样。
- `AlgoTask` 栈给 1024 words，是因为姿态解算和动作识别可能会用到较多局部变量。
- 如果后续出现 HardFault 或任务异常，优先把相关任务栈加大一档。

### 14.4 信号量、队列和互斥锁配置

在 `FREERTOS > Tasks and Queues` 中添加同步对象。这里分三类：

```text
Binary Semaphore: 用来通知“某个事件发生了”
Message Queue: 用来在任务之间传递数据
Mutex: 用来保护共享资源，防止多个任务同时访问
```

#### 14.4.1 Binary Semaphore

本项目的二值信号量都用于 DMA 或硬件事件通知，所以初始值必须为 0。

CubeMX 中添加：

| Name | Type | Initial Count | 用途 |
|---|---|---:|---|
| imuDmaDoneSem | Binary Semaphore | 0 | SPI1 DMA 读取 IMU 完成后释放，唤醒 SensorTask |
| adcHalfCpltSem | Binary Semaphore | 0 | ADC DMA 半缓冲完成，可选，用于唤醒 EmgTask |
| adcCpltSem | Binary Semaphore | 0 | ADC DMA 满缓冲完成，可选，用于唤醒 EmgTask |

为什么初始值是 0：

```text
初始值 0: 任务启动后会等待，直到中断真正释放信号量
初始值 1: 任务启动后会立刻通过，可能误以为 DMA 已经完成
```

所以 DMA 完成类信号量不要设成 1。

如果 CubeMX 没有显示 Initial Count，生成后第一次使用前可以在任务开始时先尝试 `osSemaphoreAcquire(..., 0)` 清一次，或者确认生成代码里初始 count 为 0。

#### 14.4.2 Message Queue

消息队列用于任务之间传递结构体数据。CubeMX 中常见配置项是：

```text
Queue Name
Queue Size
Item Size
Allocation
```

建议添加：

| Name | Queue Size | Item Size | Allocation | 用途 |
|---|---:|---|---|---|
| imuFrameQueue | 4 | sizeof(IMU_Frame_t) | Dynamic 或 Static | SensorTask 把 IMU 数据发给 AlgoTask |
| rehabStateQueue | 2 | sizeof(Rehab_State_t) | Dynamic 或 Static | AlgoTask 把角度/评分状态发给 DisplayTask/WirelessTask，可选 |
| debugLogQueue | 8 | sizeof(DebugLog_t) 或 sizeof(char *) | Dynamic 或 Static | 其他任务把日志发给 DebugTask，可选 |

推荐先必须建：

```text
imuFrameQueue
Queue Size: 4
Item Size: sizeof(IMU_Frame_t)
```

`IMU_Frame_t` 可以后续在代码中定义，例如：

```c
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint32_t tick;
} IMU_Frame_t;
```

队列长度为什么是 4：

```text
SensorTask 每 10ms 产生 1 帧
AlgoTask 正常应及时消费
Queue Size = 4 可以缓存约 40ms 数据
```

如果队列满了，不建议阻塞 SensorTask 太久。SensorTask 发送队列时建议：

```c
osMessageQueuePut(imuFrameQueueHandle, &imu_frame, 0, 0);
```

最后一个参数 timeout 为 0，表示队列满了就丢弃本帧，不要卡住采样任务。

`debugLogQueue` 如果用字符串指针：

```text
Item Size: sizeof(char *)
```

如果用固定结构体：

```c
typedef struct
{
    uint16_t id;
    int32_t value1;
    int32_t value2;
} DebugLog_t;
```

则：

```text
Item Size: sizeof(DebugLog_t)
```

新手阶段可以先不建 `debugLogQueue`，直接让 DebugTask 低频主动打印关键状态，系统稳定后再加日志队列。

#### 14.4.3 Mutex

Mutex 用来保护共享外设或共享数据。它和 Binary Semaphore 不一样：Mutex 表示“资源锁”，初始状态应为可用。

CubeMX 中添加：

| Name | Type | Recursive | 用途 |
|---|---|---|---|
| uart1Mutex | Mutex | Disable | 保护 USART1 发送，避免多个任务同时发数据 |
| uart2Mutex | Mutex | Disable | 保护 USART2 调试输出，避免日志交叉 |
| spi3LcdMutex | Mutex | Disable | 保护 LCD SPI3 访问，可选 |
| rehabStateMutex | Mutex | Disable | 保护全局训练状态，可选 |

Mutex 不需要设置 Initial Count。创建成功后默认是“可获取”的状态。

使用原则：

```text
谁要用共享资源，谁先 osMutexAcquire()
用完立刻 osMutexRelease()
不要长时间持有 Mutex
不要在中断里获取 Mutex
```

USART2 打印示例：

```c
osMutexAcquire(uart2MutexHandle, osWaitForever);
printf("angle=%d score=%d\r\n", angle, score);
osMutexRelease(uart2MutexHandle);
```

LCD 刷新示例：

```c
osMutexAcquire(spi3LcdMutexHandle, osWaitForever);
LCD_DrawText(0, 0, "RUN");
osMutexRelease(spi3LcdMutexHandle);
```

注意：如果只有 DisplayTask 一个任务操作 LCD，可以先不建 `spi3LcdMutex`。如果后续 DebugTask 或其他任务也要画屏，再启用它。

#### 14.4.4 当前阶段最小推荐

如果你想先少配一点，最小集合是：

```text
Binary Semaphore:
  imuDmaDoneSem, Initial Count = 0

Message Queue:
  imuFrameQueue, Queue Size = 4, Item Size = sizeof(IMU_Frame_t)

Mutex:
  uart2Mutex
```

如果 ADC DMA 和 USART1 也开始接入，再加：

```text
adcHalfCpltSem, Initial Count = 0
adcCpltSem, Initial Count = 0
uart1Mutex
```

如果开始移植 LVGL 或多个任务访问 LCD，再加：

```text
spi3LcdMutex
```
### 14.5 各任务具体做什么

#### SensorTask

职责：每 10ms 读取一次 IMU，也就是 100Hz。

流程：

```text
1. 等待 10ms 周期
2. 拉低 IMU_CS
3. 调用 HAL_SPI_TransmitReceive_DMA()
4. 等待 imuDmaDoneSem，建议超时 2ms
5. 拉高 IMU_CS
6. 解析 ax, ay, az, gx, gy, gz
7. 通过 imuFrameQueue 发给 AlgoTask
```

伪代码：

```c
void StartSensorTask(void *argument)
{
    uint32_t tick = osKernelGetTickCount();

    for (;;)
    {
        tick += 10;

        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_DMA(&hspi1, imu_tx_buf, imu_rx_buf, imu_len);

        if (osSemaphoreAcquire(imuDmaDoneSemHandle, 2) == osOK)
        {
            HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
            IMU_ParseFrame(imu_rx_buf, &imu_frame);
            osMessageQueuePut(imuFrameQueueHandle, &imu_frame, 0, 0);
        }
        else
        {
            HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
            imu_error_count++;
        }

        osDelayUntil(tick);
    }
}
```

注意：SensorTask 不做姿态解算，只负责稳定采样。

#### AlgoTask

职责：等待 IMU 数据，做姿态解算、角度计算、动作识别和评分。

流程：

```text
1. 等待 imuFrameQueue
2. 取出 IMU 数据
3. 姿态解算
4. 计算角度
5. 动作识别
6. 更新评分结果
7. 更新全局训练状态
```

伪代码：

```c
void StartAlgoTask(void *argument)
{
    IMU_Frame_t frame;

    for (;;)
    {
        if (osMessageQueueGet(imuFrameQueueHandle, &frame, NULL, osWaitForever) == osOK)
        {
            Attitude_Update(&frame);
            RehabAngle_Update();
            ActionDetect_Update();
            Score_Update();
        }
    }
}
```

#### EmgTask

职责：处理 ADC DMA 缓冲区，计算肌电参与状态。

ADC 采样由硬件自动完成：

```text
TIM3 -> ADC1 -> DMA -> emg_adc_buf
```

EmgTask 不需要每次启动 ADC，只需要定期处理缓冲区。

流程：

```text
1. 每 20ms 运行一次
2. 从 emg_adc_buf 取最近数据
3. 整流
4. 平滑
5. 计算 RMS 或平均幅值
6. 判断主动发力状态
```

#### WirelessTask

职责：USART1 数据通信。

用途：

```text
1. 上传训练数据
2. 接收手机命令
3. 接收或发送板间同步数据
```

建议发送频率：2Hz-5Hz。不要用 USART1 高频刷 IMU 原始数据。

#### DebugTask

职责：USART2 调试日志。

用途：

```text
1. printf 调试日志
2. 输出错误码
3. 临时修改阈值参数
```

建议只输出低频摘要，例如：

```text
angle=75, emg=1, score=86, imu_err=0
```

不要在高优先级任务里直接大量 printf。

#### DisplayTask

职责：LCD 显示刷新。

建议周期：100ms-200ms。

显示内容：

```text
当前模式
角度
EMG 状态
次数
评分
异常提示
```

注意：不要全屏高频刷新，显示任务优先级保持低。

#### UiTask

职责：LED、蜂鸣器和预留按键。

当前 PA12 只预留，不参与业务逻辑：

```text
PA12: GPIO_Input, Pull-Up, No EXTI
```

UiTask 当前只处理：

```text
LED_STATUS
BUZZER
```

后续如果启用 PA12，可以在 UiTask 中轮询消抖，不一定要用 EXTI。

### 14.6 中断回调怎么配合 RTOS

中断里不要做复杂计算。中断只做三类事：

```text
释放信号量
投递队列
置标志位
```

SPI1 DMA 完成回调示例：

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        osSemaphoreRelease(imuDmaDoneSemHandle);
    }
}
```

ADC DMA 回调示例：

```c
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        osSemaphoreRelease(adcHalfCpltSemHandle);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        osSemaphoreRelease(adcCpltSemHandle);
    }
}
```

重要：这些回调里调用了 RTOS API，所以对应中断优先级必须设置为 5 或更大的数字。

### 14.7 新手推荐调试顺序

不要一次性把所有任务都打开。建议按下面顺序逐步加：

1. 只创建 `DebugTask`，确认 USART2 能打印日志。
2. 创建 `SensorTask`，先用阻塞 SPI 读取 IMU `WHO_AM_I`。
3. 把 SensorTask 改成 SPI1 DMA 读取 IMU。
4. 加 `imuDmaDoneSem`，确认 DMA 完成后能唤醒 SensorTask。
5. 加 `imuFrameQueue` 和 `AlgoTask`。
6. 启动 TIM3 + ADC1 + DMA，加 `EmgTask`。
7. 加 `DisplayTask`，低频刷新 LCD。
8. 加 `WirelessTask`，用 USART1 发送训练结果。
9. 最后再处理 CST816D 触摸、PA12 按键等低优先级功能。

每加一个任务，都先观察：

```text
是否还在正常打印日志
IMU 100Hz 是否稳定
是否进入 HardFault
任务栈是否不足
```

如果出现不稳定，优先检查：

```text
中断优先级是否 >= 5
任务栈是否太小
是否在中断里 printf
是否在高优先级任务里长时间刷屏
```
## 15. 低功耗配置

MVP 演示阶段先不启用 Stop/Standby 低功耗模式，避免影响调试。

可先配置：

```text
PWR: 默认
Low Power Mode: Disable
```

后期续航优化：

- SYSCLK 从 160MHz 降到 80MHz。
- P169H002 LCD降低刷新频率到 5Hz-10Hz。
- 无线串口只在结果变化或训练结束时发送。
- 没有训练时降低 IMU ODR。

## 16. CubeMX 最小可用配置清单

第一版必须配置：

- SYS: Serial Wire
- RCC: 默认内部时钟或外部晶振
- SPI1: 本节点 IMU
- SPI2: 不启用，预留
- SPI3: P169H002 LCD 显示，ST7789T3
- ADC1: 肌电输入
- DMA: SPI1_RX 使用 GPDMA1 Channel 0、SPI1_TX 使用 GPDMA1 Channel 1、ADC1 Circular
- TIM6: 不启用算法中断，预留
- TIM3: 500Hz ADC 触发 TRGO / Update Event
- USART1: 无线数据串口接口，115200
- USART2: 无线调试串口接口，115200
- GPIO: IMU CS、LCD CS/DC/RST/BL、TP_RST/TP_INT、无线串口 EN/STATE 可选、PA12 按键预留、LED/蜂鸣器
- NVIC: SPI1 DMA=5、ADC DMA=6、USART1=7、USART2=8、TIM17=15；PA12 按键当前不启用 EXTI

## 17. 建议引脚命名

在 CubeMX 的 GPIO 页面中给引脚设置 User Label，后续代码更清晰：

```text
IMU_SCK         = PA5
IMU_MISO        = PA6
IMU_MOSI        = PA7
IMU_CS          = PA4
IMU_INT         = PC8，可选
LCD_SCL         = PC10
LCD_SDA         = PC12
LCD_SDO         = PC11，可选
LCD_CS          = PA15
LCD_DC          = PB8
LCD_RST         = PB9
LCD_BL          = PA8
TP_SCL          = PB6
TP_SDA          = PB7
TP_RST          = PC5
TP_INT          = PC4
KEY_RESERVED    = PA12
LED_STATUS      = PC6
BUZZER          = PB2
EMG_ADC         = PC0
WIRELESS_RXD    = PA9  // 接模块 RXD，STM32 USART1_TX
WIRELESS_TXD    = PA10 // 接模块 TXD，STM32 USART1_RX
WIRELESS_EN     = PB0，可选
WIRELESS_STATE  = PB1，可选
DBG_RXD         = PA2  // 接调试模块 RXD，STM32 USART2_TX
DBG_TXD         = PA3  // 接调试模块 TXD，STM32 USART2_RX
```

生成代码后可直接在程序中使用：

```c
HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
```

## 18. 生成代码后的初始化顺序

CubeMX 生成的 `main.c` 中建议保持以下初始化顺序：

```text
HAL_Init()
SystemClock_Config()
MX_GPIO_Init()
MX_DMA_Init()
MX_ADC1_Init()
MX_SPI1_Init()
MX_SPI3_Init()
MX_I2C1_Init()
MX_USART1_UART_Init()
MX_USART2_UART_Init()
MX_TIM3_Init()

应用层初始化:
  IMU_Init()
  LCD_Init()
  Touch_Init()，可选
  WirelessData_Init()
  DebugUart_Init()
  RehabAlgo_Init()

启动外设:
  osKernelInitialize()
  创建 RTOS 任务、队列、信号量
  HAL_ADC_Start_DMA()
  HAL_TIM_Base_Start(&htim3)
  HAL_UART_Receive_IT(&huart1, ...)
  HAL_UART_Receive_IT(&huart2, ...)
  osKernelStart()
```

注意：如果 ADC 由 TIM3 TRGO / Update Event 触发，需要先启动 ADC DMA，再启动 TIM3。SPI1 DMA 不在初始化阶段持续启动，而是在 `SensorTask` 每 10ms 发起一次 `HAL_SPI_TransmitReceive_DMA()`。FreeRTOS 启用后，`osKernelStart()` 之后不再返回主循环，业务逻辑放到任务里。

## 19. 调试顺序

不要一次性联调全部功能，建议按以下顺序验证：

1. 点灯和 USART2 printf 调试日志。
2. USART1 无线数据串口收发。
3. SPI1 阻塞方式读取本节点 IMU WHO_AM_I。
4. SPI1 DMA 方式连续读取 IMU 原始数据。
5. FreeRTOS `SensorTask` 10ms 周期是否稳定。
6. 本节点 IMU 100Hz 连续读取是否稳定。
7. ADC DMA 肌电采样波形是否变化。
8. LCD 显示文字。
9. USART1 无线数据串口发送训练结果。
10. USART2 无线调试串口输出角度、状态和错误码。
11. 姿态解算和角度计算。
12. 动作识别、评分和异常提醒。

## 20. 推荐通信与显示刷新频率

| 模块 | 频率 |
|---|---:|
| IMU 原始读取 | 100Hz |
| 姿态解算 | 100Hz |
| EMG ADC | 500Hz |
| EMG 特征更新 | 20Hz-50Hz |
| 动作识别 | 50Hz-100Hz |
| P169H002 LCD刷新 | 5Hz-10Hz |
| USART1 无线数据实时发送 | 2Hz-5Hz |
| USART1 无线数据结果发送 | 训练结束发送 |
| USART2 调试日志 | 2Hz-10Hz，按需开启 |

## 21. MVP 推荐最终配置摘要

```text
MCU: STM32U575RIT6
SYSCLK: 160MHz
IMU: SPI1, Mode 3, 8bit, Software NSS, PA4 CS GPIO, RX/TX DMA
Display: P169H002 LCD, SPI3, Mode 0, 8bit, Software NSS, ST7789T3, PA15 CS GPIO
EMG: ADC1 single channel, TIM3 TRGO / Update Event, 500Hz, DMA circular
Wireless Data UART: USART1, PA9/PA10, 115200, 8N1
Wireless Debug UART: USART2, PA2/PA3, 115200, 8N1
Timer:
  TIM6 = disabled/reserved, not algorithm tick
  TIM3 = 500Hz ADC trigger
GPIO:
  1 reserved key: PA12, GPIO_Input, no EXTI
  1 status LED
  1 buzzer or warning output
Interrupt:
  SPI1 RX DMA: GPDMA1 Channel 0, preemption priority 5
  SPI1 TX DMA: GPDMA1 Channel 1, preemption priority 5
  ADC DMA: preemption priority 6
  USART1: preemption priority 7
  USART2: preemption priority 8
  TIM17 HAL timebase: preemption priority 15
  PA12 key reserved, EXTI disabled
RTOS: Enable, CMSIS-RTOS v2
```

## 22. 关键注意事项

- FreeRTOS 下会调用 FromISR API 的中断优先级必须设置为 5 或更大的数字；不要把 SPI/ADC/UART 中断设为 0-4。
- SPI1 和 SPI3 使用软件片选：CubeMX 中关闭 Hardware NSS，CS 引脚按普通 GPIO_Output 配置，默认高电平；SPI1 DMA 完成前不要拉高 IMU_CS。
- 当前节点只接一颗 IMU；第二颗远端 IMU 建议由另一块 STM32U575RIT6 就近采集，再通过无线串口或其他通信方式同步数据。
- 本项目屏幕为 P169H002 LCD 触摸显示屏：ST7789T3 使用 SPI 显示接口，CST816D 使用 I2C 触摸接口，不按 I2C OLED 配置。
- 肌电模块输出电压必须在 STM32 ADC 输入范围内，不能超过 3.3V。
- 无线串口模块必须和 STM32 共地。
- USART2 是独立无线调试串口，建议只发低频摘要日志，不要在高优先级任务或中断里直接大量 printf。
- 显示刷新不要放在高优先级中断里，应在主循环低频刷新。
- 姿态解算和动作识别不要写在 TIM6 中断里，TIM6 中断只置标志位，主循环执行算法。
- 第一次联调时，先关闭显示屏大面积刷新，只打印传感器数据，确认采集稳定后再加 UI。























