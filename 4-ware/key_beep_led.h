#include <stm32f4xx.h>

#ifndef __KEY_BEEP_H__
#define __KEY_BEEP_H__

extern uint8_t key1_flag;
extern uint8_t key2_flag;

void Key_Beep_Led_Init(void);
void Key_Read(void);
void Beep_OnOff(uint8_t status);
void Led1_OnOff(uint8_t status);
void Led2_OnOff(uint8_t status);

#endif
