#include "stm32f4xx.h"

#ifndef __SPI1_H__
#define __SPI1_H__

void SPI1_NSS(uint8_t BitValue);  //NSS ctrl											  
void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(uint8_t SpeedSet); //设置SPI1速度   
uint8_t SPI1_ReadWriteByte(uint8_t TxData);//SPI1总线读写一个字节
		 
#endif

