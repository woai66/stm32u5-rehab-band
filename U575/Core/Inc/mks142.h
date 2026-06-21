#ifndef __MKS142_H__
#define __MKS142_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MKS-142 智能健康监测模块（UART 38400 8N1）实时数据包协议。
   实时包定长 88 字节，以 0xFF 打头且包内不会再出现 0xFF，故仅凭帧头即可可靠同步。
   字节偏移按 0 基下标（buf[0] 为帧头）。 */
#define MKS142_FRAME_HEAD      0xFFU
#define MKS142_FRAME_LEN       88U

#define MKS142_OFF_WAVE        1U    /* acdata[0..63] PPG 波形，int8 */
#define MKS142_OFF_WAVE_LEN    64U
#define MKS142_OFF_HEART_RATE  65U
#define MKS142_OFF_SPO2        66U
#define MKS142_OFF_MICRO_CIRC  67U   /* bk 微循环 */
#define MKS142_OFF_FATIGUE     68U   /* 疲劳指数 */
#define MKS142_OFF_SBP         71U   /* 收缩压 */
#define MKS142_OFF_DBP         72U   /* 舒张压 */
#define MKS142_OFF_SDNN        76U
#define MKS142_OFF_RMSSD       77U
#define MKS142_OFF_NN50        78U
#define MKS142_OFF_PNN50       79U

/* 指令字节（经 USART2_TX 发给模块） */
#define MKS142_CMD_ACQUIRE_ON  0x8AU /* 采集开：模块开始上报实时包，上电后必须先发此指令 */
#define MKS142_CMD_ACQUIRE_OFF 0x88U /* 采集关：停止上报 */
#define MKS142_CMD_SLEEP_ON    0x98U /* 进入休眠 */
#define MKS142_CMD_WAKE        0x00U /* 退出休眠 */

typedef struct
{
    uint8_t heart_rate;   /* 心率 bpm */
    uint8_t spo2;         /* 血氧 % */
    uint8_t micro_circ;   /* 微循环 */
    uint8_t fatigue;      /* 疲劳指数 */
    uint8_t sbp;          /* 收缩压 */
    uint8_t dbp;          /* 舒张压 */
    uint8_t sdnn;         /* 心率变异性 SDNN */
    uint8_t rmssd;        /* 心率变异性 RMSSD */
    uint8_t valid;        /* 1 表示已解析到有效帧 */
} MKS142_Data_t;

/* 流式解析上下文：从 UART 字节流中找帧头并攒满一帧 */
typedef struct
{
    uint8_t buf[MKS142_FRAME_LEN];
    uint8_t index;
    uint8_t in_frame;
} MKS142_Parser_t;

void MKS142_ParserReset(MKS142_Parser_t *parser);

/* 从一帧 88 字节缓冲中提取各字段，frame[0] 应为 0xFF */
void MKS142_ParseFrame(const uint8_t *frame, MKS142_Data_t *out);

/* 喂入一个字节，集满一帧时解析并写入 out、返回 1，否则返回 0 */
uint8_t MKS142_FeedByte(MKS142_Parser_t *parser, uint8_t byte, MKS142_Data_t *out);

#ifdef __cplusplus
}
#endif

#endif
