#include <stm32f4xx.h>

#ifndef __STEPPER_PITCH_H__
#define __STEPPER_PITCH_H__

extern float now_pitch;

void USART2_DMA_Init(void);
void USART2_DMA_Send(uint8_t* data, uint16_t len);
void Set_Pitch(uint8_t aac,uint16_t rpm,float theta);

#endif
