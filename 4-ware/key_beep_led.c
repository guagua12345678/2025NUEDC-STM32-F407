//按键蜂鸣器led

#include <stm32f4xx.h>

uint8_t key1_flag = 0;
uint8_t key2_flag = 0;

void Key_Beep_Led_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOC, GPIO_Pin_5 | GPIO_Pin_14 | GPIO_Pin_15);
}

//定时刷新就行 如果按下会置按键标志位 需要手动清零按键标志
void Key_Read(void)
{
	static uint8_t last_key1_state = 1;
	static uint8_t last_key2_state = 1;
	uint8_t now_key1_state = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
	uint8_t now_key2_state = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
	if(now_key1_state && last_key1_state==0) key1_flag = 1;
	if(now_key2_state && last_key2_state==0) key2_flag = 1;
	last_key1_state = now_key1_state;
    last_key2_state = now_key2_state;
}

void Beep_OnOff(uint8_t status)
{
	if(status) GPIO_SetBits(GPIOC, GPIO_Pin_5);
	else GPIO_ResetBits(GPIOC, GPIO_Pin_5);
}

void Led1_OnOff(uint8_t status)
{
	if(status) GPIO_SetBits(GPIOC, GPIO_Pin_15);
	else GPIO_ResetBits(GPIOC, GPIO_Pin_15);
}

void Led2_OnOff(uint8_t status)
{
	if(status) GPIO_SetBits(GPIOC, GPIO_Pin_14);
	else GPIO_ResetBits(GPIOC, GPIO_Pin_14);
}
