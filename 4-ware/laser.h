#include "stm32f4xx.h"                  // Device header

#ifndef __LASER_H__
#define __LASER_H__

void Init_Laser(void);
void Laser_3V3_OnOff(uint8_t status);
void Laser_5V_OnOff(uint8_t status);

#endif
