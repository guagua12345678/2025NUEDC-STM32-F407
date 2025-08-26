#include "stm32f4xx.h"                  // Device header

#ifndef __DELAY_H__
#define __DELAY_H__

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t sec);

#endif
