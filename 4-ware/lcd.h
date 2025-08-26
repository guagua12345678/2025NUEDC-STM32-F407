#include <stm32f4xx.h>

#ifndef __LCD_H__
#define __LCD_H__

void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);//填充区域
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//画线
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//画矩形框
void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//画实心矩形框
void LCD_Draw_Triangel(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 color);//画三角形
void LCD_Fill_Triangel(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 color);//画实心三角形
void LCD_Circle(int xc,int yc,int r,int fill,u16 color);

void LCD_ShowChar(u16 x,u16 y,u16 fc,u16 bc,char num,u8 size,u8 mode);
void LCD_ShowStr(u16 x,u16 y,u16 fc,u16 bc,char *str,u8 size,u8 mode);
void LCD_ShowNum(u16 x,u16 y,u16 fc,u16 bc,u32 num,u8 len,u8 size,u8 mode);
void LCD_ShowSignedNum(u16 x, u16 y, u16 fc, u16 bc, int32_t num, u8 len, u8 size, u8 mode);
void LCD_ShowFloatNum(u16 x, u16 y, u16 fc, u16 bc, float num, u8 int_len, u8 dec_len, u8 size, u8 mode);

void LCD_DrawFont16(u16 x, u16 y, u16 fc, u16 bc, char *s,u8 mode);
void LCD_DrawFont24(u16 x, u16 y, u16 fc, u16 bc, char *s,u8 mode);
void LCD_DrawFont32(u16 x, u16 y, u16 fc, u16 bc, char *s,u8 mode);

void LCD_Drawbmp16(u16 x,u16 y,const unsigned char *p);

#endif
