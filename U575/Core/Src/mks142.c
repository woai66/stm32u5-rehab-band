#include "mks142.h"

/* MKS-142 实时数据包协议解析层，不涉及具体 UART/HAL，便于独立测试与复用。
   UART 接收与采集指令发送由上层任务（HeartRateTask）用 HAL 完成后调用本层。 */

void MKS142_ParserReset(MKS142_Parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->index = 0U;
    parser->in_frame = 0U;
}

void MKS142_ParseFrame(const uint8_t *frame, MKS142_Data_t *out)
{
    if ((frame == NULL) || (out == NULL))
    {
        return;
    }

    out->heart_rate = frame[MKS142_OFF_HEART_RATE];
    out->spo2 = frame[MKS142_OFF_SPO2];
    out->micro_circ = frame[MKS142_OFF_MICRO_CIRC];
    out->fatigue = frame[MKS142_OFF_FATIGUE];
    out->sbp = frame[MKS142_OFF_SBP];
    out->dbp = frame[MKS142_OFF_DBP];
    out->sdnn = frame[MKS142_OFF_SDNN];
    out->rmssd = frame[MKS142_OFF_RMSSD];
    out->valid = 1U;
}

uint8_t MKS142_FeedByte(MKS142_Parser_t *parser, uint8_t byte, MKS142_Data_t *out)
{
    if ((parser == NULL) || (out == NULL))
    {
        return 0U;
    }

    if (parser->in_frame == 0U)
    {
        /* 等待帧头：实时包保证仅帧头为 0xFF，凭此可靠对齐 */
        if (byte == MKS142_FRAME_HEAD)
        {
            parser->buf[0] = byte;
            parser->index = 1U;
            parser->in_frame = 1U;
        }
        return 0U;
    }

    /* 帧内不应再出现 0xFF：若出现，说明此前丢字节/噪声导致错位，按新帧头重新对帧 */
    if (byte == MKS142_FRAME_HEAD)
    {
        parser->buf[0] = byte;
        parser->index = 1U;
        return 0U;
    }

    parser->buf[parser->index] = byte;
    parser->index++;

    if (parser->index >= MKS142_FRAME_LEN)
    {
        MKS142_ParseFrame(parser->buf, out);
        parser->index = 0U;
        parser->in_frame = 0U;
        return 1U;
    }

    return 0U;
}
