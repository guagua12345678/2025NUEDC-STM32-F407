#include "stm32f4xx.h"

#ifndef __W25Q128_H__
#define __W25Q128_H__		    

extern uint16_t W25QXX_TYPE;

void W25QXX_Init(void);
uint16_t  W25QXX_ReadID(void);  	    		//读取FLASH ID
uint8_t	 W25QXX_ReadSR(void);        			//读取状态寄存器 
void W25QXX_Write_SR(uint8_t sr);  				//写状态寄存器
void W25QXX_Write_Enable(void);  				//写使能 
void W25QXX_Write_Disable(void);				//写保护
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);   //读取flash
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);//写入flash
void W25QXX_Erase_Chip(void);    	  			//整片擦除
void W25QXX_Erase_Sector(uint32_t Dst_Addr);	//扇区擦除
void W25QXX_Wait_Busy(void);           			//等待空闲
void W25QXX_PowerDown(void);        			//进入掉电模式
void W25QXX_WAKEUP(void);						//唤醒

#endif
















