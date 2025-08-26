#include <stm32f4xx.h>

#ifndef __TIMER_CLOCK_H__
#define __TIMER_CLOCK_H__

void TimClock_Init(void);
void TimClock_Start(uint8_t num);
float TimeClock_Stop(uint8_t num);

#endif
