//2.8inch lcd触摸屏代码，有参考中景园电子屏幕相关代码

#include "lcd_init.h"
#include "string.h"
#include "lcd_font.h" 
#include "delay.h"
#include "lcd.h"

void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{  	
	u16 i,j;			
	u16 width=ex-sx+1;
	u16 height=ey-sy+1;
	LCD_SetWindows(sx,sy,ex,ey);
	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		Lcd_WriteData_16Bit(color);
	}
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}

void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol,color);//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}
} 

void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}  

void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_Fill(x1,y1,x2,y2,color);
}
 
void _draw_circle_8(int xc, int yc, int x, int y, u16 color)
{
	LCD_DrawPoint(xc + x, yc + y, color);
	LCD_DrawPoint(xc - x, yc + y, color);
	LCD_DrawPoint(xc + x, yc - y, color);
	LCD_DrawPoint(xc - x, yc - y, color);
	LCD_DrawPoint(xc + y, yc + x, color);
	LCD_DrawPoint(xc - y, yc + x, color);
	LCD_DrawPoint(xc + y, yc - x, color);
	LCD_DrawPoint(xc - y, yc - x, color);
}

void LCD_Circle(int xc, int yc,int r, int fill,u16 color)
{
	int x = 0, y = r, yi, d;
	d = 3 - 2 * r;
	if (fill) 
	{
		// 如果填充（画实心圆）
		while (x <= y) {
			for (yi = x; yi <= y; yi++)
				_draw_circle_8(xc, yc, x, yi, color);

			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	} else 
	{
		while (x <= y) {
			_draw_circle_8(xc, yc, x, y, color);
			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	}
}

void LCD_Draw_Triangel(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	LCD_DrawLine(x0,y0,x1,y1,color);
	LCD_DrawLine(x1,y1,x2,y2,color);
	LCD_DrawLine(x2,y2,x0,y0,color);
}

static void _swap(u16 *a, u16 *b)
{
	u16 tmp;
	tmp = *a;
	*a = *b;
	*b = tmp;
}

void LCD_Fill_Triangel(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 a, b, y, last;
	int dx01, dy01, dx02, dy02, dx12, dy12;
	long sa = 0;
	long sb = 0;
 	if (y0 > y1) 
	{
		_swap(&y0,&y1); 
		_swap(&x0,&x1);
 	}
 	if (y1 > y2) 
	{
		_swap(&y2,&y1); 
		_swap(&x2,&x1);
 	}
	if (y0 > y1) 
	{
		_swap(&y0,&y1); 
		_swap(&x0,&x1);
	}
	if(y0 == y2) 
	{ 
		a = b = x0;
		if(x1 < a)
		{
			a = x1;
		}
		else if(x1 > b)
		{
			b = x1;
		}
		if(x2 < a)
		{
			a = x2;
		}
		else if(x2 > b)
		{
			b = x2;
		}
		LCD_Fill(a,y0,b,y0,color);
		return;
	}
	dx01 = x1 - x0;
	dy01 = y1 - y0;
	dx02 = x2 - x0;
	dy02 = y2 - y0;
	dx12 = x2 - x1;
	dy12 = y2 - y1;
	
	if(y1 == y2)
	{
		last = y1; 
	}
	else
	{
		last = y1-1; 
	}
	for(y=y0; y<=last; y++) 
	{
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		if(a > b)
		{
			_swap(&a,&b);
		}
		LCD_Fill(a,y,b,y,color);
	}
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for(; y<=y2; y++) 
	{
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		if(a > b)
		{
			_swap(&a,&b);
		}
		LCD_Fill(a,y,b,y,color);
	}
}

void LCD_ShowChar(u16 x,u16 y,u16 fc,u16 bc,char num,u8 size,u8 mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;
	uint16_t x0=x;
	sizex=size/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*size;
	num=num-' ';
	LCD_SetWindows(x,y,x+sizex-1,y+size-1);
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(size==12)temp=ascii_1206[num][i];
		else if(size==16)temp=ascii_1608[num][i];
		else if(size==24)temp=ascii_2412[num][i];
		else if(size==32)temp=ascii_3216[num][i];
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)
			{
				if(temp&(0x01<<t))Lcd_WriteData_16Bit(fc);
				else Lcd_WriteData_16Bit(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

void LCD_ShowStr(u16 x,u16 y,u16 fc,u16 bc,char *str,u8 size,u8 mode)
{
    while((*str<='~')&&(*str>=' '))
    {   
		if(x>(lcddev.width-1)||y>(lcddev.height-1)) 
		return;     
        LCD_ShowChar(x,y,fc,bc,*str,size,mode);
        x+=size/2;
        str++;
    }
} 

u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

void LCD_ShowNum(u16 x,u16 y,u16 fc,u16 bc,u32 num,u8 len,u8 size,u8 mode)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,fc,bc,' ',size,mode);
				continue;
			}else enshow=1;
		}
	 	LCD_ShowChar(x+(size/2)*t,y,fc,bc,temp+'0',size,mode);
	}
} 


void LCD_ShowSignedNum(u16 x, u16 y, u16 fc, u16 bc, s32 num, u8 len, u8 size, u8 mode)
{
    u8 sign = 0;
    u32 abs_num;
	
    if (num < 0) {sign = 1;abs_num = -num;} 
	else {abs_num = num;}
	
    if (sign) LCD_ShowChar(x, y, fc, bc, '-', size, mode);
    else LCD_ShowChar(x, y, fc, bc, '+', size, mode);
	
    LCD_ShowNum(x + size/2, y, fc, bc, abs_num, len-1, size, mode);
}

void LCD_ShowFloatNum(u16 x, u16 y, u16 fc, u16 bc, float num, u8 int_len, u8 dec_len, u8 size, u8 mode)
{
    s32 int_part;
    u32 dec_part;
    u8 sign = 0;
    float abs_num;

    if (num < 0) {sign = 1;abs_num = -num;} 
	else abs_num = num;

    int_part = (s32)abs_num;
    dec_part = (u32)((abs_num - int_part) * mypow(10, dec_len) + 0.5f);  
    
    if (sign) LCD_ShowChar(x, y, fc, bc, '-', size, mode);
	else LCD_ShowChar(x, y, fc, bc, '+', size, mode);  
    x += size/2;
    
    LCD_ShowNum(x, y, fc, bc, int_part, int_len-1, size, mode);
    x += (int_len-1) * (size/2);
    
    LCD_ShowChar(x, y, fc, bc, '.', size, mode);
    x += size/2;

    u8 t;
    for (t = 0; t < dec_len; t++) 
	{
        u8 digit = (dec_part / mypow(10, dec_len-t-1)) % 10;
        LCD_ShowChar(x + (size/2)*t, y, fc, bc, digit + '0', size, mode);
    }
}

void LCD_DrawFont16(u16 x, u16 y, u16 fc, u16 bc, char *s,u8 mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;
	uint16_t TypefaceNum;
	uint16_t x0=x;
	TypefaceNum=(16/8+((16%8)?1:0))*16;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_SetWindows(x,y,x+16-1,y+16-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))Lcd_WriteData_16Bit(fc);
						else Lcd_WriteData_16Bit(bc);
						m++;
						if(m%16==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==16)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;
	}
}

void LCD_DrawFont24(u16 x, u16 y, u16 fc, u16 bc, char *s,u8 mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(24/8+((24%8)?1:0))*24;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_SetWindows(x,y,x+24-1,y+24-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))Lcd_WriteData_16Bit(fc);
						else Lcd_WriteData_16Bit(bc);
						m++;
						if(m%24==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==24)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

void LCD_DrawFont32(u16 x, u16 y, u16 fc, u16 bc, char *s,u8 mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(32/8+((32%8)?1:0))*32;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_SetWindows(x,y,x+32-1,y+32-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))Lcd_WriteData_16Bit(fc);
						else Lcd_WriteData_16Bit(bc);
						m++;
						if(m%32==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==32)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

void LCD_Drawbmp16(u16 x,u16 y,const unsigned char *p)
{
  	int i; 
	unsigned char picH,picL; 
	LCD_SetWindows(x,y,x+40-1,y+40-1);
    for(i=0;i<40*40;i++)
	{	
	 	picL=*(p+i*2);
		picH=*(p+i*2+1);				
		Lcd_WriteData_16Bit(picH<<8|picL);  						
	}	
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
}
