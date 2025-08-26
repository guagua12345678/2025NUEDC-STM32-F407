//使用systick定时器作为软件延时器

#include "stm32f4xx.h"                  // Device header

void Delay_us(uint32_t us)
{
	SysTick->LOAD = 168 * us;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

void Delay_ms(uint32_t ms)
{
	while(ms--)
	{
		Delay_us(1000);
	}
}

void Delay_s(uint32_t sec)
{
	while(sec--)
	{
		Delay_ms(1000);
	}
} 
