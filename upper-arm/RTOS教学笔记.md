# RTOS 教学笔记

这份文档用于记录 U575 工程里的 FreeRTOS 学习笔记、调试方法和后续说明。以后如果说“保存到文案里”，就继续追加到这里。

## 1. 从裸机思维切换到 RTOS 思维

裸机程序通常把主要业务写在 `main.c` 的 `while (1)` 里：

```c
while (1)
{
  read_sensor();
  printf("data\r\n");
  HAL_Delay(100);
}
```

当前 U575 工程启用了 FreeRTOS。`main.c` 执行到：

```c
osKernelStart();
```

之后，CPU 由 RTOS 调度器接管，正常不会再回到 `main.c` 后面的主循环。因此业务代码应该写到任务函数里，而不是写在 `main.c` 的 `while (1)` 中。

## 2. 任务是什么

任务可以理解成“同时运行的多个小主循环”。每个任务都有自己的 `for (;;)`：

```c
void StartSensorTask(void *argument)
{
  for (;;)
  {
    read_sensor();
    osDelay(5);
  }
}
```

`osDelay(5)` 表示当前任务休息 5ms，把 CPU 让给其他任务。RTOS 任务里一般用 `osDelay()`，不要用 `HAL_Delay()`。

## 3. 当前工程的任务分工

当前工程里比较重要的任务：

- `SensorTask`：负责采集传感器，比如 LSM6DSR 陀螺仪/加速度计。
- `DebugTask`：适合放串口打印调试代码。
- `DisplayTask`：负责屏幕和触摸测试。
- `AlgoTask`：后续适合放康复动作识别、滤波、算法逻辑。
- `WirelessTask`：后续适合放无线通信逻辑。

原则：采样任务尽量短、快、稳定；打印、显示、无线发送这些慢操作放到单独任务中。

## 4. 队列是什么

队列可以理解成任务之间传数据的“邮箱”。

`SensorTask` 采集 IMU 后，把数据放进队列：

```c
osMessageQueuePut(imuFrameQueueHandle, &imu_data, 0U, 0U);
```

`DebugTask` 从队列取出数据再打印：

```c
osMessageQueueGet(imuFrameQueueHandle, &imu_data, NULL, 1000);
```

这样 `SensorTask` 不需要直接负责打印，采样不会被串口输出拖慢。

## 5. IMU 原始数据打印方法

工程里已经添加了 `printf` 重定向，输出到 `USART2`：

- PA2：USART2_TX，接 USB-TTL 的 RX。
- PA3：USART2_RX，接 USB-TTL 的 TX。
- 波特率：115200。
- 串口格式：8N1。

工程里也封装了 IMU 原始数据打印接口：

```c
void LSM6DSR_PrintRaw(const LSM6DSR_Data_t *data);
```

推荐在 `DebugTask` 中这样使用：

```c
void StartDebugTask(void *argument)
{
  /* USER CODE BEGIN DebugTask */
  LSM6DSR_Data_t imu_data;

  for (;;)
  {
    if (osMessageQueueGet(imuFrameQueueHandle, &imu_data, NULL, 1000) == osOK)
    {
      osMutexAcquire(uart2MutexHandle, osWaitForever);
      LSM6DSR_PrintRaw(&imu_data);
      osMutexRelease(uart2MutexHandle);
    }

    osDelay(100);
  }
  /* USER CODE END DebugTask */
}
```

不要在 DMA 回调、中断回调里 `printf`，否则容易阻塞或影响实时性。

## 6. 三条基本规则

1. 不要把业务代码写到 `main.c` 的 `while (1)` 里。
2. 不要在中断回调或 DMA 回调里 `printf`。
3. RTOS 任务里延时优先用 `osDelay()`，不要用 `HAL_Delay()`。
