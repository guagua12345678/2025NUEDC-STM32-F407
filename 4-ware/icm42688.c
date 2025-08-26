//陀螺仪icm42688标准库解算

#include "stm32f4xx.h"
#include "icm42688.h"
#include "delay.h"
#include "ahrs.h"
#include "timer_clock.h"
#include "lcd.h"
#include "lcd_init.h"

#define Dgr2Rad 57.29578f

// 软件SPI引脚定义
#define SPI_NSS_PIN    GPIO_Pin_0   // PE0
#define SPI_SCLK_PIN   GPIO_Pin_13  // PE13
#define SPI_MOSI_PIN   GPIO_Pin_14  // PE14
#define SPI_MISO_PIN   GPIO_Pin_15  // PE15
#define SPI_GPIO_PORT  GPIOE

// 外部中断引脚定义
#define EXTI_PIN       GPIO_Pin_2   // PE2
#define EXTI_PORT      GPIOE
#define EXTI_PORT_SRC  GPIO_PortSourceGPIOE
#define EXTI_PIN_SRC   GPIO_PinSource2
#define EXTI_IRQn      EXTI2_IRQn

#define accel_dead_band 0.003f
#define gyro_dead_band 0.06f

volatile uint8_t icm42688_ok = 0;

int16_t real_a_x = 0;
int16_t real_a_y = 0;
int16_t real_a_z = 0;
int16_t real_g_x = 0;
int16_t real_g_y = 0;
int16_t real_g_z = 0;

int16_t offset_a_x = 0;
int16_t offset_a_y = 0;
int16_t offset_a_z = 0;
int16_t offset_g_x = 0;
int16_t offset_g_y = 0;
int16_t offset_g_z = 0;

float tempeture = 0.0f;
float a_x = 0.0f;
float a_y = 0.0f;
float a_z = 0.0f;
float g_x = 0.0f;
float g_y = 0.0f;
float g_z = 0.0f;

//延时
void Spi_Delay_40ns(void)
{
	__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();
}

// 软件SPI初始化函数
void SoftSPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = SPI_NSS_PIN | SPI_SCLK_PIN | SPI_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_SetBits(SPI_GPIO_PORT, SPI_NSS_PIN);
    GPIO_ResetBits(SPI_GPIO_PORT, SPI_SCLK_PIN);
}

// PE2外部中断初始化
void EXTI_PE2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Pin = EXTI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(EXTI_PORT, &GPIO_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);

    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

// 软件SPI传输函数
uint8_t SoftSPI_Transfer(uint8_t data)
{
    uint8_t received = 0;
    
    for(int i = 0; i < 8; i++)
    {
        if(data & 0x80) GPIO_SetBits(SPI_GPIO_PORT, SPI_MOSI_PIN);
        else GPIO_ResetBits(SPI_GPIO_PORT, SPI_MOSI_PIN);
        data <<= 1;

        Spi_Delay_40ns();
        GPIO_SetBits(SPI_GPIO_PORT, SPI_SCLK_PIN);
		
        Spi_Delay_40ns();
        received <<= 1;
        if(GPIO_ReadInputDataBit(SPI_GPIO_PORT, SPI_MISO_PIN))
            received |= 0x01;
		
        Spi_Delay_40ns();
        GPIO_ResetBits(SPI_GPIO_PORT, SPI_SCLK_PIN);
        Spi_Delay_40ns();
    }
    return received;
}

// SPI写寄存器函数
void SoftSPI_WriteReg(uint8_t reg, uint8_t value)
{
    GPIO_ResetBits(SPI_GPIO_PORT, SPI_NSS_PIN);
    SoftSPI_Transfer(reg & 0x7F);
    SoftSPI_Transfer(value);
    GPIO_SetBits(SPI_GPIO_PORT, SPI_NSS_PIN);
}

// SPI读寄存器函数
uint8_t SoftSPI_ReadReg(uint8_t reg)
{
    uint8_t value;
    GPIO_ResetBits(SPI_GPIO_PORT, SPI_NSS_PIN);
    SoftSPI_Transfer(reg | 0x80);
    value = SoftSPI_Transfer(0x00);
    GPIO_SetBits(SPI_GPIO_PORT, SPI_NSS_PIN);
    return value;
}

//spi连续读
void SoftSPI_ReadRegs(uint8_t startReg, uint8_t *data, uint32_t len)
{
    GPIO_ResetBits(SPI_GPIO_PORT, SPI_NSS_PIN);
    SoftSPI_Transfer(startReg | 0x80);
    for(uint32_t i = 0; i < len; i++)
    {
        data[i] = SoftSPI_Transfer(0x00);
    }
    GPIO_SetBits(SPI_GPIO_PORT, SPI_NSS_PIN);
}

//我是谁
uint8_t ICM42688_WhoAmI(void)
{
	uint8_t temp = 0;
	SoftSPI_ReadRegs(ICM42688_WHO_AM_I,&temp,1); 
	if(temp == 0x47) return 1;
	else return 0;
}

void ICM42688_Gyro_Settings(uint8_t dps, uint8_t freq)
{
    uint8_t config = 0;

    if (dps <= 7) config |= (dps << 5);
	else config |= (0 << 5);
	
	if(freq <= 11 || freq == 15) config |= freq;
	else config |= 6;

    SoftSPI_WriteReg(ICM42688_GYRO_CONFIG0, config);
}

void ICM42688_Accel_Settings(uint8_t range, uint8_t freq)
{
    uint8_t config = 0;
    
    if(range <= 3) config |= (range << 5);
    else config |= (0x00 << 5);

	if(freq <= 15) config |= freq;
	else config |= 6;

    SoftSPI_WriteReg(ICM42688_ACCEL_CONFIG0, config);
}

void ICM42688_Gyro_Filter_Settings(uint8_t temp_bw, uint8_t ui_filt_ord)
{
    uint8_t config = 0;

    if(temp_bw <= 6) config |= (temp_bw << 5);
    else config |= (0 << 5);

    if(ui_filt_ord <= 2) config |= (ui_filt_ord << 2);
    else config |= (0 << 2);
    
    config |= 0x02;
    
    SoftSPI_WriteReg(ICM42688_GYRO_CONFIG1, config);
}

void ICM42688_Filter_BW_Settings(uint8_t accel_bw, uint8_t gyro_bw)
{
    uint8_t config = 0;
    
    if(accel_bw <= 7 || accel_bw ==14 || accel_bw ==15) config |= (accel_bw << 4);
    else config |= (0x01 << 4);
    
    if(gyro_bw <= 7 || gyro_bw ==14 || gyro_bw <=15) config |= gyro_bw;
    else config |= 0x01;
    
    SoftSPI_WriteReg(ICM42688_GYRO_ACCEL_CONFIG0, config);
}

void ICM42688_Accel_Filter_Order(uint8_t ui_filt_ord)
{
    uint8_t config = 0;
    
    if(ui_filt_ord <= 2) config |= (ui_filt_ord << 3);
    else config |= (0x01 << 3);

    config |= 0x04;
   
    SoftSPI_WriteReg(ICM42688_ACCEL_CONFIG1, config);
}

void ICM42688_Calibration(void)
{
	static uint8_t count = 0;
	static int32_t total_a_x = 0;
	static int32_t total_a_y = 0;
	static int32_t total_a_z = 0;
	static int32_t total_g_x = 0;
	static int32_t total_g_y = 0;
	static int32_t total_g_z = 0;

	while(++count <= 20)
	{
		total_a_x += real_a_x;
		total_a_y += real_a_y;
		total_a_z += real_a_z;
		total_g_x += real_g_x;
		total_g_y += real_g_y;
		total_g_z += real_g_z;
		Delay_ms(50);
	}
	offset_a_x = total_a_x/20;
	offset_a_y = total_a_y/20;
	offset_a_z = total_a_z/20 - 8192.0f;
	offset_g_x = total_g_x/20;
	offset_g_y = total_g_y/20;
	offset_g_z = total_g_z/20;
}

u8 ICM42688_Init(u8 G_dps,u8 A_rge,u8 G_freq,u8 A_freq,u8 G_bw_filt,u8 A_bw_filt,u8 T_bw_filt,u8 G_ui_filt,u8 A_ui_filt)
{
	uint8_t res = 0;
	SoftSPI_Init();
	EXTI_PE2_Init();
	if(ICM42688_WhoAmI()) res = 1;
	else res = 0;
	if(res)
	{
		SoftSPI_WriteReg(ICM42688_DEVICE_CONFIG,0x01);//软复位
		Delay_ms(2);
		SoftSPI_WriteReg(ICM42688_INT_CONFIG,0x00);//下降沿 开漏 脉冲
		Delay_ms(2);
		SoftSPI_WriteReg(ICM42688_INT_SOURCE0,0x08);//使能中断
		Delay_ms(2);
		SoftSPI_WriteReg(ICM42688_PWR_MGMT0,0x0F);//开温度 陀螺 加速度 高性能
		Delay_ms(2);
		ICM42688_Gyro_Settings(G_dps,G_freq);
		Delay_ms(2);
		ICM42688_Accel_Settings(A_rge,A_freq);
		Delay_ms(2);
		ICM42688_Gyro_Filter_Settings(T_bw_filt,G_ui_filt);
		Delay_ms(2);
		ICM42688_Accel_Filter_Order(A_ui_filt);
		Delay_ms(2);
		ICM42688_Filter_BW_Settings(A_bw_filt,G_bw_filt);
		Delay_ms(2);
		
		Delay_ms(500);
		ICM42688_Calibration();
		Delay_ms(500);
		ahrs_init();
		icm42688_ok = 1;
	}
	return res;
}

void ICM42688_Read(void)
{
	uint8_t buf[14];
	SoftSPI_ReadRegs(ICM42688_TEMP_DATA1,buf,14);
	
	int16_t temp = (buf[0] << 8) | buf[1];
	tempeture = (temp / 132.48) + 25;
	real_a_x = (buf[2] << 8) | buf[3];
	real_a_y = (buf[4] << 8) | buf[5];
	real_a_z = (buf[6] << 8) | buf[7];
	
	real_g_x = (buf[8]  << 8) | buf[9];
	real_g_y = (buf[10] << 8) | buf[11];
	real_g_z = (buf[12] << 8) | buf[13];
	
	a_x = (float)(real_a_x - offset_a_x)/8192.0f;
	a_y = (float)(real_a_y - offset_a_y)/8192.0f;
	a_z = (float)(real_a_z - offset_a_z)/8192.0f;
	g_x = (float)(real_g_x - offset_g_x)/65.5f/Dgr2Rad;
	g_y = (float)(real_g_y - offset_g_y)/65.5f/Dgr2Rad;
	g_z = (float)(real_g_z - offset_g_z)/65.5f/Dgr2Rad;
}

void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
		float dt = TimeClock_Stop(0) / 1000.0f; //s
		TimClock_Start(0);
		SoftSPI_ReadReg(ICM42688_INT_STATUS);
		ICM42688_Read();
		if(icm42688_ok) ahrs_update(g_x, g_y, g_z, a_x, a_y, a_z, dt);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
