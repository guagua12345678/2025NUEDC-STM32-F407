#include <stm32f4xx.h>

#ifndef __STEPPER_YAW_H__
#define __STEPPER_YAW_H__

extern float now_yaw;

void USART3_DMA_Init(void);
void USART3_DMA_Send(uint8_t* data, uint16_t len);
void Set_Yaw(uint8_t aac,uint16_t rpm,float theta);

#endif
