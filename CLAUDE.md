# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概况

这是一个智能康复臂带/康复手表方向的 STM32 嵌入式项目与备赛资料集合。主要固件工程位于 `U575/`，目标芯片为 `STM32U575RITx`，使用 STM32CubeMX 生成初始化代码，并用 Keil MDK-ARM 工程维护编译配置。

根目录还包含方案文档、硬件资料和 LCD/触摸屏参考资料：

- `备赛/智能康复臂带-MVP方案.md`：当前 MVP 目标，核心是 STM32U575 本地完成 IMU/EMG 采集、动作识别、角度计算、评分、显示和蓝牙输出。
- `project-ai建议.md`：更早的方案分级和比赛定位建议。
- `lcd/P183B001-CTP/`：ST7789P3 LCD、CST816T 触摸芯片的数据手册、参考驱动和屏幕规格资料。

## 常用命令

当前仓库不是 git 仓库，且未发现 Makefile、CMake、PlatformIO、包管理脚本或自动化测试工程。主要构建入口是 Keil MDK 工程：

```bash
# 使用 Keil UV4 命令行批量编译，需本机已安装 Keil MDK 并把 UV4.exe 加入 PATH
UV4.exe -b U575/MDK-ARM/U575.uvprojx -t U575 -j0
```

如果 `UV4.exe` 不在 PATH，需要使用本机实际安装路径，例如：

```bash
"/c/Keil_v5/UV4/UV4.exe" -b U575/MDK-ARM/U575.uvprojx -t U575 -j0
```

生成目标配置：

- Keil target：`U575`
- 工程文件：`U575/MDK-ARM/U575.uvprojx`
- 输出目录：`U575/MDK-ARM/U575/`
- 工程配置已开启 HEX 输出。

目前没有发现单元测试或单测试命令；验证通常依赖 Keil 编译、下载到 STM32U575 硬件后观察 LCD/触摸、串口、传感器和任务运行状态。执行烧录、串口监视或外设交互前先向用户确认。

## 固件结构

`U575/` 是 STM32CubeMX/Keil 生成的主工程：

- `U575/U575.ioc`：CubeMX 配置源，记录芯片、引脚、时钟、外设、FreeRTOS 对象等。
- `U575/Core/Inc` 与 `U575/Core/Src`：用户应用代码和 CubeMX 生成的初始化代码。
- `U575/Drivers/`：CMSIS 与 STM32U5 HAL 驱动。
- `U575/Middlewares/Third_Party/FreeRTOS/`：FreeRTOS 与 CMSIS-RTOS2 适配层。
- `U575/MDK-ARM/`：Keil 工程、启动文件和 IDE 配置。

启动流程：

1. `Core/Src/main.c` 调用 `HAL_Init()`、`SystemClock_Config()`，再依次初始化 GPIO、GPDMA、ICACHE、ADC1、SPI1、TIM3、I2C1、SPI3、USART1、USART2。
2. `main.c` 调用 `osKernelInitialize()` 与 `MX_FREERTOS_Init()`。
3. `Core/Src/app_freertos.c` 创建 CMSIS-RTOS2 任务、队列、互斥量和信号量，然后调度器启动。
4. 当前只有 `DisplayTask` 执行实际测试逻辑：调用 `LCD_Touch_TestTask()`，初始化 LCD 和 CST816T 触摸后显示彩条并轮询触摸点；其他任务目前是占位循环。

## FreeRTOS 任务与通信对象

`app_freertos.c` 中已生成面向康复臂带功能拆分的任务骨架：

- `SensorTask`：预留给 IMU/传感器采集，优先级 High。
- `AlgoTask`：预留给姿态解算、动作识别、评分算法，优先级 AboveNormal。
- `EmgTask`：预留给 EMG 采集和主动参与判定，优先级 AboveNormal。
- `WirelessTask`：预留给蓝牙/无线串口输出。
- `UiTask`：预留给按键、触摸和模式切换。
- `DisplayTask`：当前运行 LCD + 触摸测试。
- `DebugTask`：预留给调试日志输出。

已生成的队列/同步对象包括：

- `imuFrameQueue`：4 个元素，每个 48 字节，预留传感器帧传递。
- `rehabStateQueue`：2 个元素，每个 32 字节，预留训练状态传递。
- `debugLogQueue`：8 个元素，每个 64 字节，预留调试日志。
- `imuDmaDoneSem`、`adcHalfCpltSem`、`adcCpltSem`：预留 DMA/ADC 完成通知。
- `uart1Mutex`、`uart2Mutex`、`spi3LcdMutex`、`rehabStateMutex`：预留串口、LCD SPI 和训练状态共享保护。

中断优先级需要符合 `FreeRTOSConfig.h` 中 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`：优先级数值小于 5 的中断不要调用 FreeRTOS ISR API。

## 外设分工

当前 CubeMX 配置和代码中的主要外设用途：

- `SPI1`：IMU 总线，PA5/PA6/PA7，`IMU_ARM_CS` 在 PA4，配置为 Mode 3，约 5 Mbit/s，并配置 GPDMA TX/RX。
- `ADC1`：EMG 输入，PC0/`EMG_ADC`，12 位，TIM3 TRGO 触发，配置 GPDMA1 Channel2 循环链表。
- `TIM3`：ADC 外部触发源，当前 Prescaler `16000-1`、Period `20-1`。
- `SPI3`：LCD 总线，PC10/PC11/PC12，约 40 Mbit/s。
- `I2C1`：触摸芯片 CST816T，总线 PB6/PB7，Fast 模式。
- `USART1`：无线/蓝牙串口，PA9/PA10，115200 8N1。
- `USART2`：调试串口，PA2/PA3，115200 8N1。
- GPIO：LCD 控制脚 `LCD_CS`/`LCD_DC`/`LCD_RST`/`LCD_BL`，触摸 `TP_INT`/`TP_RST`，以及 `RUN_LED`、`LED_STATUS`、`BUZZER`、`USER_KEY`。

## LCD 与触摸驱动

当前手写驱动位于：

- `Core/Src/lcd_st7789.c`、`Core/Inc/lcd_st7789.h`
- `Core/Src/touch_cst816t.c`、`Core/Inc/touch_cst816t.h`
- `Core/Src/lcd_touch_test.c`、`Core/Inc/lcd_touch_test.h`

LCD 驱动使用阻塞式 `HAL_SPI_Transmit(&hspi3, ...)`，屏幕尺寸宏为 `LCD_WIDTH = 240`、`LCD_HEIGHT = 284`，颜色格式为 RGB565。触摸驱动使用 CST816T 的 8-bit I2C 地址 `0x2A`，轮询读取手势、点数和坐标。`LCD_Touch_TestTask()` 内部是无限循环，因此当前 `DisplayTask` 调用后不会返回。

如果后续多个任务同时访问 LCD，请使用已生成的 `spi3LcdMutex` 保护 SPI3/LCD 访问；当前驱动内部尚未加锁。

## CubeMX 与代码生成注意事项

项目包含 `U575/U575.ioc`。涉及引脚、时钟、外设、FreeRTOS 对象或中断优先级变更时，应先说明需要改哪些 CubeMX 配置以及原因，得到确认后再继续。不要直接修改 `.ioc` 或重新生成代码，除非用户明确授权。

修改 CubeMX 生成文件时，优先把用户代码放在 `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` 区域内，避免后续重新生成时丢失。手写驱动文件如 `lcd_st7789.c`、`touch_cst816t.c` 不属于 CubeMX 模板，可按现有风格直接维护。

## 当前实现状态

当前固件更像“外设打通阶段”：启动 FreeRTOS 后已能运行 LCD 彩条和触摸点测试；IMU、EMG、算法、无线、UI、调试任务的 RTOS 对象已经生成，但任务体仍是占位循环。实现 MVP 功能时应优先沿用这些任务边界和队列/信号量，而不是重写整体架构。
