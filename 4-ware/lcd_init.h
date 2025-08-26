#include <stm32f4xx.h>
#include "stdlib.h"

#ifndef __LCD_INIT_H__
#define __LCD_INIT_H__		

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     
#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 

#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入
#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入
#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //输出 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //输入

//LCD重要参数集
typedef struct
{										    
	u16  width;			//LCD 宽度
	u16  height;		//LCD 高度
	u16  id;			//LCD ID
	u8   dir;			//横屏还是竖屏控制：0竖屏；1横屏。	
	u16	 wramcmd;		//开始写gram指令
	u16  setxcmd;		//设置x坐标指令
	u16  setycmd;		//设置y坐标指令	 
}_lcd_dev;

extern _lcd_dev lcddev;

#define USE_HORIZONTAL 1//定义液晶屏顺时针旋转方向 0-0度旋转 1-90度旋转 2-180度旋转 3-270度旋转

//定义LCD的尺寸
#define LCD_W 240
#define LCD_H 320

// 更新RST引脚为PB2
#define LED  11       //背光控制引脚
#define CS   12       //片选引脚
#define RS    8       //寄存器/数据选择引脚  
#define RST   2       //复位引脚PB2

#define	LCD_LED PAout(LED)		 
#define LCD_CS  PBout(CS)
#define LCD_RS  PAout(RS)
#define LCD_RST PBout(RST)

#define	LCD_CS_SET  LCD_CS=1  
#define	LCD_RS_SET	LCD_RS=1  
#define	LCD_RST_SET	LCD_RST=1 
		    
#define	LCD_CS_CLR  LCD_CS=0  
#define	LCD_RS_CLR	LCD_RS=0  
#define	LCD_RST_CLR	LCD_RST=0 

//画笔颜色
#define WHITE       	0xFFFF
#define BLACK      		0x0000	  
#define BLUE       		0x001F  
#define BRED        	0XF81F
#define GRED 			0XFFE0
#define GBLUE			0X07FF
#define RED         	0xF800
#define MAGENTA     	0xF81F
#define GREEN       	0x07E0
#define CYAN        	0x7FFF
#define YELLOW      	0xFFE0
#define BROWN 			0XBC40 //棕色
#define BRRED 			0XFC07 //棕红色
#define GRAY  			0X8430 //灰色
#define DARKBLUE      	0X01CF //深蓝色
#define LIGHTBLUE      	0X7D7C //浅蓝色  
#define GRAYBLUE       	0X5458 //灰蓝色
#define LIGHTGREEN     	0X841F //浅绿色
#define LIGHTGRAY       0XEF5B //浅灰色
	    															  
void LCD_Init(void);
void Lcd_WriteData_16Bit(u16 Data);
void LCD_DrawPoint(u16 x,u16 y,u16 color);
void LCD_SetWindows(u16 xStar,u16 yStar,u16 xEnd,u16 yEnd);

#endif
