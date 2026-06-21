# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概况

这是一个智能康复臂带/康复手表方向的 STM32 嵌入式项目与备赛资料集合。主要固件工程位于 `U575/`，目标芯片为 `STM32U575RITx`，使用 STM32CubeMX 生成初始化代码，并用 Keil MDK-ARM 工程维护编译配置。

固件已演进为**双节点无线架构**：本工程是**腕部节点（康复手表）**，板上带一块 ST7789V 240×280 电容触摸屏，做 IMU 采集与姿态解算后，通过 USART1 把姿态帧无线发给**上臂节点**；上臂节点（接收端）尚未在本仓库实现。注意 MVP 方案文档里写的 0.96 寸 OLED 已不采用，实际显示屏是 ST7789V 240×280 LCD。

根目录还包含方案文档、硬件资料和 LCD/触摸屏参考资料：

- `备赛/智能康复臂带-MVP方案.md`：MVP 目标，核心是双 IMU + 简化 EMG，本地完成动作识别、角度计算、评分、显示和蓝牙输出。
- `备赛/智能康复臂带-CubeMX配置方案.md`、`备赛/硬件清单.md`、`备赛/人体佩戴与模块朝向规范.md`：硬件与配置参考。
- `project-ai建议.md`：更早的方案分级和比赛定位建议。
- `lcd/P183B001-CTP/`：LCD、CST816T 触摸芯片的数据手册、参考驱动和屏幕规格资料。

## 常用命令

本仓库已是 git 仓库，远程为 `github.com/woai66/stm32u5-rehab-band`（原 `HZCU-Debug` 已迁移至此），主分支 `master`。未发现 Makefile、CMake、PlatformIO、包管理脚本或自动化测试工程。主要构建入口是 Keil MDK 工程：

```bash
# 使用 Keil UV4 命令行批量编译，需本机已安装 Keil MDK 并把 UV4.exe 加入 PATH
UV4.exe -b U575/MDK-ARM/wrist.uvprojx -t wrist -j0
```

如果 `UV4.exe` 不在 PATH，需要使用本机实际安装路径，例如：

```bash
"/c/Keil_v5/UV4/UV4.exe" -b U575/MDK-ARM/wrist.uvprojx -t wrist -j0
```

生成目标配置：

- Keil target：`wrist`
- 工程文件：`U575/MDK-ARM/wrist.uvprojx`（原 `U575.uvprojx` 已改名）
- 输出目录：`U575/MDK-ARM/wrist/`
- 工程配置已开启 HEX 输出。

下载方式：板子支持 SWD（ST-Link，PA13/PA14，需 Connect Under Reset）与 ISP（板载 USB 转串口，进 Bootloader 时 `J6` 跳线帽必须跳到 `USB` 侧）。目前没有单元测试；验证依赖 Keil 编译、下载到硬件后观察 LCD/触摸、UART4 串口、IMU 和任务运行状态。执行烧录、串口监视或外设交互前先向用户确认。

## 协作与 Git 流程

本项目多人协作（owner + co-developer）。Git 约定：

- 不直接向 `master` 提交或推送；每个特性从最新 `master` 切新分支（`feat/xxx`、`fix/xxx`）。
- 分支完成后 push 并发起 PR，经队友 review 再合并到 `master`。
- 合并后本地 `git checkout master && git pull` 同步，再开下一个分支。
- 提交信息用简化 Conventional Commits（`feat`/`fix`/`docs`/`refactor`/`test`/`chore`），一个提交只做一件事。
- 未经用户明确要求，不执行 push、force-push、rebase、amend。

## 固件结构

`U575/` 是 STM32CubeMX/Keil 生成的主工程：

- `U575/wrist.ioc`：CubeMX 配置源（原 `U575.ioc` 已改名为 `wrist.ioc`），记录芯片、引脚、时钟、外设、FreeRTOS 对象等。
- `U575/Core/Inc` 与 `U575/Core/Src`：用户应用代码和 CubeMX 生成的初始化代码。
- `U575/Drivers/`：CMSIS 与 STM32U5 HAL 驱动。
- `U575/Middlewares/Third_Party/FreeRTOS/`：FreeRTOS 与 CMSIS-RTOS2 适配层（内存管理用 heap_4）。
- `U575/MDK-ARM/`：Keil 工程、启动文件和 IDE 配置。

手写的应用模块（非 CubeMX 模板）：

- `lcd_st7789.*`、`touch_cst816t.*`、`lcd_touch_test.*`：LCD、触摸和测试。
- `lsm6dsr.*`：LSM6DSR/IMU660RB 六轴 IMU 驱动（SPI1，含 DMA 读）。
- `imu_processing.*`：姿态解算（低通、零偏校准、互补滤波四元数/欧拉角、相对姿态求角度）。
- `wireless_link.*`：腕部无线帧（24 字节）打包/解析。
- `emg_sensor.*`：EMG 采集与主动参与判定（本节点暂未使用）。

启动流程：

1. `Core/Src/main.c` 调用 `HAL_Init()`、`SystemClock_Config()`，再依次初始化 GPIO、GPDMA、ICACHE、ADC1、SPI1、TIM3、I2C1、SPI3、USART1、UART4。
2. `main.c` 调用 `osKernelInitialize()` 与 `MX_FREERTOS_Init()`。
3. `Core/Src/app_freertos.c` 创建 CMSIS-RTOS2 任务、队列、互斥量和信号量，然后调度器启动。

## FreeRTOS 任务与通信对象

`app_freertos.c` 中面向腕部节点功能拆分的任务：

- `SensorTask`（已实现，High）：SPI1 读 LSM6DSR，先做陀螺零偏校准，再用 `imu_processing` 做互补滤波姿态解算，结果经 `taskENTER_CRITICAL` 发布到共享变量 `wrist_imu_raw/euler/quat`、`wrist_imu_valid`。
- `WirelessTask`（已实现）：用 `WirelessLink_BuildWristFrame` 打 24 字节腕部帧，在 `uart1Mutex` 保护下 `HAL_UART_Transmit(&huart1, ...)` 发给上臂节点，约 50Hz。
- `DisplayTask`（测试中）：背光自检闪烁 → `LCD_Init()` + `CST816T_Init()` 并打印状态 → 调 `LCD_Touch_TestTask()`（彩条 + 触摸点测试，内部死循环）。
- `DebugTask`（已实现）：读共享 IMU，经 UART4 `printf` 输出（运行期 printf 多已注释，栈 1024×4）。
- `EmgTask`：本节点无 EMG，`EmgTaskHandle = NULL` 未启动（任务函数保留）。
- `AlgoTask`、`UiTask`、`defaultTask`：仍为占位 `osDelay` 循环。

并发约定：

- 跨任务共享的 `wrist_imu_*` 用 `taskENTER_CRITICAL`/`taskEXIT_CRITICAL` 拷贝读写。
- newlib 非重入（`configUSE_NEWLIB_REENTRANT` 默认 0）：所有 `printf` 必须先持 `uart2Mutex`，否则多任务输出会冲突。
- 中断优先级需符合 `FreeRTOSConfig.h` 中 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`：优先级数值小于 5 的中断不要调用 FreeRTOS ISR API。

已生成的队列/同步对象（多数尚未接入实际逻辑）：`imuFrameQueue`、`rehabStateQueue`、`debugLogQueue`、`imuDmaDoneSem`、`adcHalfCpltSem`、`adcCpltSem`，以及互斥量 `uart1Mutex`、`uart2Mutex`、`spi3LcdMutex`、`rehabStateMutex`。

## 外设分工

当前 CubeMX 配置和代码中的主要外设用途：

- `SPI1`：IMU（LSM6DSR）总线，PA5/PA6/PA7，`IMU_ARM_CS` 在 PA4，Mode 3，含 GPDMA TxRx 及完成回调。
- `ADC1`：EMG 输入，已从 `PC0/ADC_CHANNEL_1` 改到 **`PB0/ADC_CHANNEL_15`**，12 位，TIM3 TRGO 触发，配置为 DMA 循环；**本节点未启用**（未调用 `HAL_ADC_Start_DMA`）。
- `TIM3`：ADC 外部触发源，Prescaler `16000-1`、Period `20-1`。
- `SPI3`：LCD 总线，PC10/PC11/PC12，配置为 `1LINE` 半双工、`BaudRatePrescaler=8`（约 20 Mbit/s）、软件 NSS。
- `I2C1`：触摸芯片 CST816T，总线 PB6/PB7，Fast 模式。
- `USART1`：无线串口，PA9/PA10，115200 8N1，已实际发送腕部帧。
- `UART4`：调试串口，**PA0/PA1**（原 USART2/PA2,PA3 已弃用），115200 8N1；`printf` 经 `fputc`/`__io_putchar` 重定向到 `huart4`。
- GPIO：LCD 控制脚 `LCD_CS`/`LCD_DC`/`LCD_RST`/`LCD_BL`，触摸 `TP_INT`/`TP_RST`，以及 `RUN_LED`、`LED_STATUS`、`BUZZER`、`USER_KEY`。

## LCD 与触摸驱动

当前手写驱动位于：

- `Core/Src/lcd_st7789.c`、`Core/Inc/lcd_st7789.h`
- `Core/Src/touch_cst816t.c`、`Core/Inc/touch_cst816t.h`
- `Core/Src/lcd_touch_test.c`、`Core/Inc/lcd_touch_test.h`

LCD 驱动使用阻塞式 `HAL_SPI_Transmit(&hspi3, ...)`，屏幕尺寸宏为 `LCD_WIDTH = 240`、`LCD_HEIGHT = 280`（ST7789V 240×280），颜色格式 RGB565；背光为低电平有效（`LCD_SetBacklight`）。`LCD_Init()` 返回 `HAL_StatusTypeDef`，可用 `LCD_GetLastStatus()` 取最近一次 SPI 状态。当前绘图原语只有 `LCD_Fill`/`LCD_FillScreen`/`LCD_DrawPoint`/`LCD_DrawRect`，**尚无文字/字库绘制**（做屏上 UI 时需先补 `LCD_DrawChar`/`LCD_DrawString`）。LCD 和触摸的初始化已从 `LCD_Touch_TestTask()` 移到 `DisplayTask`。触摸驱动使用 CST816T 的 8-bit I2C 地址 `0x2A`，轮询读取手势、点数和坐标。

如果后续多个任务同时访问 LCD，请使用已生成的 `spi3LcdMutex` 保护 SPI3/LCD 访问；当前驱动内部尚未加锁。若 240×280 显示有整体偏移，需要在 `LCD_SetAddress()` 的 RASET 里加 Y offset。

## CubeMX 与代码生成注意事项

CubeMX 配置源是 `U575/wrist.ioc`。涉及引脚、时钟、外设、FreeRTOS 对象或中断优先级变更时，应先说明需要改哪些 CubeMX 配置以及原因，得到确认后再继续。不要直接修改 `.ioc` 或重新生成代码，除非用户明确授权。

修改 CubeMX 生成文件时，优先把用户代码放在 `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` 区域内，避免后续重新生成时丢失。手写驱动与应用模块（`lcd_st7789.c`、`touch_cst816t.c`、`lsm6dsr.c`、`imu_processing.c`、`wireless_link.c`、`emg_sensor.c`）不属于 CubeMX 模板，可按现有风格直接维护。

## 当前实现状态

外设与「单腕节点的感知 + 遥测」已打通，康复应用层尚未开始，整体约 25–30%（约对应 MVP 第 1 周 + 部分姿态解算）。

已完成：

- 烧录链路（SWD + ISP，ISP 时 J6 跳 USB）。
- LCD ST7789V 点亮（彩条/边框/触摸点测试，高度已修为 280）。
- 触摸 CST816T 调通（读 ID/坐标，坐标方向已修正，读失败连续 5 次自动复位 I2C+芯片恢复）。
- UART4 调试串口 + printf 重定向。
- IMU LSM6DSR 采集 + 零偏校准 + 姿态解算（单 IMU）。
- 无线 USART1 发腕部四元数帧（仅发送端；接收端/上臂节点未建）。

未开始：屏上真实 UI（现仅测试图案）、双 IMU 肘关节角度（缺第二 IMU）、EMG 主动参与（缺传感器）、4 类动作识别 + 评分（`AlgoTask` 占位）、模式切换/触摸 UI（`UiTask` 占位）、蓝牙/手机端（缺模块）。

下一阶段（在手硬件 = U575 + 触摸屏 + 单个 IMU660RB，可单机完成）：把 `DisplayTask` 的彩条测试换成**屏上实时 IMU UI**，显示本机姿态/角度随手腕转动实时变化，验证 采集→解算→显示 全链路。第一步是给 LCD 驱动补字库与 `LCD_DrawString`，再让 `DisplayTask` 读共享 `wrist_imu_euler` 刷新显示（用 `IMUProc_EulerToX10` 取 ×10 整数，避免浮点 printf）。实现 MVP 功能时应沿用现有任务边界与队列/信号量，而不是重写整体架构。
