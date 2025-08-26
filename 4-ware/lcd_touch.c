//屏幕触摸初始化

#include "lcd_touch.h" 
#include "lcd_init.h"
#include "delay.h"
#include "stdlib.h"
#include "math.h"
#include "lcd.h"	    

u16 Touch_X = 0;
u16 Touch_Y = 0;
u16 x = 0;
u16 y = 0;
u8 Lcd_Touch_Flag = 0;

float xfac = 0.065f;
float yfac = -0.089f;
short xoff = -15;
short yoff = 350;

u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;

// 硬件SPI发送接收函数
u8 SPI3_SendByte(u8 byte)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI3, byte);
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI3);
}

/* 从触摸屏控制器读取AD转换值 */
u16 TP_Read_AD(u8 CMD)	  
{ 	 
    u16 Num = 0;
    u8 high_byte, low_byte;
    
    TCS = 0; // 片选使能
    
    SPI3_SendByte(CMD); // 发送命令
    Delay_us(6); // 保持原延时
    
    // 接收16位数据
    high_byte = SPI3_SendByte(0x00);
    low_byte = SPI3_SendByte(0x00);
    Num = (high_byte << 8) | low_byte;
    Num >>= 3; // 取12位有效数据
    
    TCS = 1; // 片选禁止
    return Num;
}

#define READ_TIMES 5 	
#define LOST_VAL 1	

/* 读取X或Y坐标值（带滤波处理） */
u16 TP_Read_XOY(u8 xy)
{
	u16 i, j;
	u16 buf[READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<READ_TIMES;i++)buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<READ_TIMES-1; i++)
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
} 

/* 读取原始X,Y坐标值 */
u8 TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(CMD_RDX);
	ytemp=TP_Read_XOY(CMD_RDY);	  												   	
	*x=xtemp;
	*y=ytemp;
	return 1;
}

#define ERR_RANGE 50 

/* 读取两次坐标并取平均值（提高精度） */
u8 TP_Read_XY2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	u8 flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else return 0;	  
} 

/* 触摸扫描函数，返回触摸状态 */
void TP_Scan(void)
{
    if(PEN == 0)
	{
		Lcd_Touch_Flag = 1;
		TP_Read_XY2(&x, &y);
        Touch_X = xfac * x + xoff;
        Touch_Y = yfac * y + yoff;
    }
	else
	{
		Lcd_Touch_Flag = 0;
		
		Touch_X = 0;
		Touch_Y = 0;
		x = 0xffff;
		y = 0xffff;
    }
}
	 
/* 触摸屏初始化函数 */
u8 TP_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    // 1. 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | 
                           RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    // 2. 初始化INT和CS引脚
    // INT (PA12)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // CS (PA4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    TCS = 1; // 默认禁止片选

    // 3. 配置SPI引脚 (PB4-MISO, PB5-MOSI, PC10-SCK)
    // SCK (PC10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    
    // MOSI (PB5)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);
    
    // MISO (PB4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);

    // 4. 配置SPI3
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       // CPOL=0
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;     // CPHA=0
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        // 软件控制NSS
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // 适当分频
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI3, &SPI_InitStructure);
    SPI_Cmd(SPI3, ENABLE);

    return 1;
}
