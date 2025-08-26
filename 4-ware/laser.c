//控制激光开关的模拟开关

#include <stm32f4xx.h>

void Init_Laser(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;       // 输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 高速
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // 无上/下拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    GPIO_ResetBits(GPIOC, GPIO_Pin_5);
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

void Laser_3V3_OnOff(uint8_t status)
{
	if(status) GPIO_SetBits(GPIOC, GPIO_Pin_5);
	else GPIO_ResetBits(GPIOC, GPIO_Pin_5);
}

void Laser_5V_OnOff(uint8_t status)
{
	if(status) GPIO_SetBits(GPIOD, GPIO_Pin_14);
	else GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

