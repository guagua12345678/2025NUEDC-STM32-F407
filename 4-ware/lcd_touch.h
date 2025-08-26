#ifndef __LCD_TOUCH_H__
#define __LCD_TOUCH_H__

#include <stm32f4xx.h>

#define TP_PRES_DOWN 0x80
#define TP_CATH_PRES 0x40

// 删除结构体定义
#define PEN        PAin(12)      // INT
#define DOUT       PBin(4)       // MISO
#define TDIN       PBout(5)      // MOSI
#define TCLK       PCout(10)     // SCK
#define TCS        PAout(4)      // CS

// 声明全局变量
extern u16 Touch_X;
extern u16 Touch_Y;
extern u8 Lcd_Touch_Flag;

void TP_Scan(void);
u8 TP_Init(void);

#endif
