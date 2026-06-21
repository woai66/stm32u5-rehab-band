#ifndef __VIBRATION_MOTOR_H__
#define __VIBRATION_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Vibration_Init(void);
void Vibration_On(void);
void Vibration_Off(void);
void Vibration_Set(uint8_t enable);
void Vibration_PulseBlocking(uint32_t on_ms);
void Vibration_PatternBlocking(uint32_t on_ms, uint32_t off_ms, uint8_t repeat);

#ifdef __cplusplus
}
#endif

#endif /* __VIBRATION_MOTOR_H__ */
