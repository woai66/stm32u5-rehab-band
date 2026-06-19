/**
 ******************************************************************************
 * @file    lsm6dsr.c
 * @brief   LSM6DSR IMU传感器驱动程序实现
 *          支持SPI通信，包括阻塞模式和DMA模式
 *          集成FreeRTOS信号量用于DMA同步
 ******************************************************************************
 */

/* ==================== Include ==================== */
#include "lsm6dsr.h"
#include "app_freertos.h"
#include "spi.h"
#include <stdio.h>
/* ==================== 宏定义 ==================== */
#define LSM6DSR_SPI_TIMEOUT_MS       (10U)
#define LSM6DSR_BOOT_DELAY_MS        (20U)
#define LSM6DSR_RESET_TIMEOUT_MS     (100U)
#define LSM6DSR_DMA_FRAME_LEN        (13U)
#define LSM6DSR_RAW_DATA_LEN         (12U)

#define LSM6DSR_SPI_READ             (0x80U)
#define LSM6DSR_WHO_AM_I             (0x0FU)
#define LSM6DSR_CTRL1_XL             (0x10U)
#define LSM6DSR_CTRL2_G              (0x11U)
#define LSM6DSR_CTRL3_C              (0x12U)
#define LSM6DSR_CTRL4_C              (0x13U)
#define LSM6DSR_CTRL6_C              (0x15U)
#define LSM6DSR_CTRL7_G              (0x16U)
#define LSM6DSR_CTRL9_XL             (0x18U)
#define LSM6DSR_OUTX_L_G             (0x22U)

#define LSM6DSR_WHO_AM_I_VALUE       (0x6BU)

#define LSM6DSR_CTRL3_SW_RESET       (0x01U)
#define LSM6DSR_CTRL3_BDU_IF_INC     (0x44U)
#define LSM6DSR_CTRL4_I2C_DISABLE    (0x04U)
#define LSM6DSR_CTRL9_I3C_DISABLE    (0x02U)

/* 208 Hz ODR for a 5 ms sensor task, +/-8 g and +/-2000 dps ranges. */
#define LSM6DSR_CTRL1_XL_208HZ_8G    (0x5CU)
#define LSM6DSR_CTRL2_G_208HZ_2000   (0x5CU)

#define LSM6DSR_ACC_SENS_G_PER_LSB   (0.000244f)
#define LSM6DSR_GYRO_SENS_DPS_PER_LSB (0.070f)

/* ==================== 静态变量 ==================== */
static uint8_t lsm6dsr_dma_tx[LSM6DSR_DMA_FRAME_LEN];
static uint8_t lsm6dsr_dma_rx[LSM6DSR_DMA_FRAME_LEN];
static volatile HAL_StatusTypeDef lsm6dsr_dma_status = HAL_OK;
static volatile uint8_t lsm6dsr_dma_busy = 0U;

/**
 * @brief 选中LSM6DSR芯片 (拉低CS引脚)
 */
static void LSM6DSR_Select(void)
{
  HAL_GPIO_WritePin(IMU_ARM_CS_GPIO_Port, IMU_ARM_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 取消选中LSM6DSR芯片 (拉高CS引脚)
 */
static void LSM6DSR_Deselect(void)
{
  HAL_GPIO_WritePin(IMU_ARM_CS_GPIO_Port, IMU_ARM_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 向指定寄存器写入单个字节
 * @param reg   寄存器地址
 * @param value 要写入的数据
 * @return HAL状态码
 */
static HAL_StatusTypeDef LSM6DSR_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg, value};
  HAL_StatusTypeDef status;

  LSM6DSR_Select();
  status = HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), LSM6DSR_SPI_TIMEOUT_MS);
  LSM6DSR_Deselect();

  return status;
}

/**
 * @brief 从指定寄存器读取多个字节
 * @param reg  起始寄存器地址
 * @param data 数据接收缓冲区指针
 * @param len  要读取的字节数
 * @return HAL状态码
 */
static HAL_StatusTypeDef LSM6DSR_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;
  uint8_t addr = reg | LSM6DSR_SPI_READ;

  LSM6DSR_Select();
  status = HAL_SPI_Transmit(&hspi1, &addr, 1U, LSM6DSR_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, data, len, LSM6DSR_SPI_TIMEOUT_MS);
  }
  LSM6DSR_Deselect();

  return status;
}

/**
 * @brief 解析原始数据字节为物理量
 * @details 将小端序(Little-Endian)的原始字节转换为有符号整数，
 *          并乘以灵敏度系数转换为物理单位 (g 和 dps)
 * @param raw  原始数据字节数组 (来自传感器)
 * @param data 解析后的数据结构指针
 */
static void LSM6DSR_ParseRaw(const uint8_t *raw, LSM6DSR_Data_t *data)
{
  data->gyro_x = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
  data->gyro_y = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
  data->gyro_z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
  data->acc_x = (int16_t)((uint16_t)raw[7] << 8 | raw[6]);
  data->acc_y = (int16_t)((uint16_t)raw[9] << 8 | raw[8]);
  data->acc_z = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);

  data->acc_g_x = (float)data->acc_x * LSM6DSR_ACC_SENS_G_PER_LSB;
  data->acc_g_y = (float)data->acc_y * LSM6DSR_ACC_SENS_G_PER_LSB;
  data->acc_g_z = (float)data->acc_z * LSM6DSR_ACC_SENS_G_PER_LSB;
  data->gyro_dps_x = (float)data->gyro_x * LSM6DSR_GYRO_SENS_DPS_PER_LSB;
  data->gyro_dps_y = (float)data->gyro_y * LSM6DSR_GYRO_SENS_DPS_PER_LSB;
  data->gyro_dps_z = (float)data->gyro_z * LSM6DSR_GYRO_SENS_DPS_PER_LSB;
}

/* ==================== 公共API函数 ==================== */
/**
 * @brief 读取传感器ID (WHO_AM_I)
 * @return 返回读取到的ID值，正常应为 0x6B
 */
uint8_t LSM6DSR_ReadWhoAmI(void)
{
  uint8_t who_am_i = 0U;

  (void)LSM6DSR_ReadRegs(LSM6DSR_WHO_AM_I, &who_am_i, 1U);
  return who_am_i;
}

/**
 * @brief 初始化LSM6DSR传感器
 * @details 执行步骤:
 *          1. 等待上电稳定
 *          2. 校验设备ID
 *          3. 软件复位并等待复位完成
 *          4. 配置基本控制寄存器 (BDU, IF_INC, 禁用I2C/I3C)
 *          5. 配置加速度计和陀螺仪的量程及ODR
 * @return HAL状态码
 */
HAL_StatusTypeDef LSM6DSR_Init(void)
{
  uint32_t start_tick;
  uint8_t ctrl3;

  HAL_Delay(LSM6DSR_BOOT_DELAY_MS);
  LSM6DSR_Deselect();

  if (LSM6DSR_ReadWhoAmI() != LSM6DSR_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }

  if (LSM6DSR_WriteReg(LSM6DSR_CTRL3_C, LSM6DSR_CTRL3_SW_RESET) != HAL_OK)
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  do
  {
    if (LSM6DSR_ReadRegs(LSM6DSR_CTRL3_C, &ctrl3, 1U) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if ((ctrl3 & LSM6DSR_CTRL3_SW_RESET) == 0U)
    {
      break;
    }
  } while ((HAL_GetTick() - start_tick) < LSM6DSR_RESET_TIMEOUT_MS);

  if ((ctrl3 & LSM6DSR_CTRL3_SW_RESET) != 0U)
  {
    return HAL_TIMEOUT;
  }

  if (LSM6DSR_WriteReg(LSM6DSR_CTRL3_C, LSM6DSR_CTRL3_BDU_IF_INC) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (LSM6DSR_WriteReg(LSM6DSR_CTRL4_C, LSM6DSR_CTRL4_I2C_DISABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (LSM6DSR_WriteReg(LSM6DSR_CTRL9_XL, LSM6DSR_CTRL9_I3C_DISABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (LSM6DSR_WriteReg(LSM6DSR_CTRL1_XL, LSM6DSR_CTRL1_XL_208HZ_8G) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (LSM6DSR_WriteReg(LSM6DSR_CTRL2_G, LSM6DSR_CTRL2_G_208HZ_2000) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (LSM6DSR_WriteReg(LSM6DSR_CTRL6_C, 0x00U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (LSM6DSR_WriteReg(LSM6DSR_CTRL7_G, 0x00U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief 通过SPI阻塞模式读取原始传感器数据
 * @param data 存储解析后数据的结构体指针
 * @return HAL状态码
 */
HAL_StatusTypeDef LSM6DSR_ReadRaw(LSM6DSR_Data_t *data)
{
  uint8_t raw[LSM6DSR_RAW_DATA_LEN];

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  if (LSM6DSR_ReadRegs(LSM6DSR_OUTX_L_G, raw, sizeof(raw)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  LSM6DSR_ParseRaw(raw, data);

  return HAL_OK;
}

/**
 * @brief 通过SPI DMA模式读取原始传感器数据 (配合FreeRTOS)
 * @param data       存储解析后数据的结构体指针
 * @param timeout_ms 等待DMA完成的超时时间 (毫秒)
 * @return HAL状态码
 */
HAL_StatusTypeDef LSM6DSR_ReadRawDma(LSM6DSR_Data_t *data, uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;

  if ((data == NULL) || (imuDmaDoneSemHandle == NULL) || (lsm6dsr_dma_busy != 0U))
  {
    return HAL_ERROR;
  }

  while (osSemaphoreAcquire(imuDmaDoneSemHandle, 0U) == osOK)
  {
  }

  lsm6dsr_dma_tx[0] = LSM6DSR_OUTX_L_G | LSM6DSR_SPI_READ;
  for (uint32_t i = 1U; i < LSM6DSR_DMA_FRAME_LEN; i++)
  {
    lsm6dsr_dma_tx[i] = 0xFFU;
    lsm6dsr_dma_rx[i] = 0U;
  }
  lsm6dsr_dma_rx[0] = 0U;
  lsm6dsr_dma_status = HAL_BUSY;
  lsm6dsr_dma_busy = 1U;

  LSM6DSR_Select();
  status = HAL_SPI_TransmitReceive_DMA(&hspi1, lsm6dsr_dma_tx, lsm6dsr_dma_rx, LSM6DSR_DMA_FRAME_LEN);
  if (status != HAL_OK)
  {
    lsm6dsr_dma_busy = 0U;
    lsm6dsr_dma_status = status;
    LSM6DSR_Deselect();
    return status;
  }

  if (osSemaphoreAcquire(imuDmaDoneSemHandle, timeout_ms) != osOK)
  {
    (void)HAL_SPI_Abort(&hspi1);
    lsm6dsr_dma_busy = 0U;
    lsm6dsr_dma_status = HAL_TIMEOUT;
    LSM6DSR_Deselect();
    return HAL_TIMEOUT;
  }

  if (lsm6dsr_dma_status != HAL_OK)
  {
    return lsm6dsr_dma_status;
  }

  LSM6DSR_ParseRaw(&lsm6dsr_dma_rx[1], data);
  return HAL_OK;
}

/**
 * @brief 打印原始传感器数据到串口
 * @param data 包含传感器数据的结构体指针
 */
void LSM6DSR_PrintRaw(const LSM6DSR_Data_t *data)
{
  if (data == NULL)
  {
    return;
  }

  printf("ACC raw:  x=%6d y=%6d z=%6d\r\n",
         data->acc_x, data->acc_y, data->acc_z);
  printf("GYRO raw: x=%6d y=%6d z=%6d\r\n",
         data->gyro_x, data->gyro_y, data->gyro_z);
}

/* ==================== HAL 回调函数 ==================== */

/**
 * @brief SPI DMA 传输完成回调
 * @param hspi SPI句柄指针
 * @note 此函数在中断上下文中执行，应尽量简短
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi->Instance == SPI1) && (lsm6dsr_dma_busy != 0U))
  {
    lsm6dsr_dma_busy = 0U;
    lsm6dsr_dma_status = HAL_OK;
    LSM6DSR_Deselect();
    if (imuDmaDoneSemHandle != NULL)
    {
      (void)osSemaphoreRelease(imuDmaDoneSemHandle);
    }
  }
}

/**
 * @brief SPI 错误回调
 * @param hspi SPI句柄指针
 * @note 此函数在中断上下文中执行
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if ((hspi->Instance == SPI1) && (lsm6dsr_dma_busy != 0U))
  {
    lsm6dsr_dma_busy = 0U;
    lsm6dsr_dma_status = HAL_ERROR;
    LSM6DSR_Deselect();
    if (imuDmaDoneSemHandle != NULL)
    {
      (void)osSemaphoreRelease(imuDmaDoneSemHandle);
    }
  }
}
