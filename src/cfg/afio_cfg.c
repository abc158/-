//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*by liyongyong*/
#include "afio.h"


/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  adc 采样通道配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t adc_afio[] =
{
    {GPIO_A,AFIO_PIN_ALL},
    {GPIO_B,AFIO_PIN_0 | AFIO_PIN_1| AFIO_PIN_2 | AFIO_PIN_3 | AFIO_PIN_4},
    {IO_NULL,IO_NULL},	
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  SPI0 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t spi0_afio[] =
{
    {GPIO_C,AFIO_PIN_10},
    {GPIO_F,AFIO_PIN_13|AFIO_PIN_14|AFIO_PIN_15},
    {IO_NULL,IO_NULL},  
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  SPI1 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t spi1_afio[] =
{
    {GPIO_F,AFIO_PIN_4 | AFIO_PIN_5 | AFIO_PIN_6 |AFIO_PIN_7 },//gyro 
    {IO_NULL,IO_NULL},	
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  UART0 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t uart0_afio[] =
{
  {GPIO_D,AFIO_PIN_12|AFIO_PIN_13},  //wifi
  {IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  UART1 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t uart1_afio[] =
{
        {GPIO_E,AFIO_PIN_1|AFIO_PIN_2},  //debug
	{IO_NULL,IO_NULL},  	
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  UART2 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t uart2_afio[] =
{
        {GPIO_E,AFIO_PIN_7|AFIO_PIN_8},  //ui
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  UART3 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t uart3_afio[] =
{
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  I2C0 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t i2c0_afio[] =
{
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  I2C1 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t i2c1_afio[] =
{
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  PID 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t pid_afio[] =
{
        {GPIO_C,AFIO_PIN_7},//MOTOR L
        {GPIO_G,AFIO_PIN_0},//MOTOR R        
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  MCTM0 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t mctm0_afio[] =
{
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  MCTM1 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t mctm1_afio[] =
{
        {GPIO_F,AFIO_PIN_8},//charge
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  GPTM0 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t gptm0_afio[] =
{
        {GPIO_D,AFIO_PIN_7|AFIO_PIN_8},//vac PWM, RX
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  GPTM1 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t gptm1_afio[] =
{
        {GPIO_D,AFIO_PIN_14 | AFIO_PIN_15},//main/side
	{IO_NULL,IO_NULL},	
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  GPTM2 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t gptm2_afio[] =
{
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  IR 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t ir_afio[] =
{
    {IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  LED 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t led_afio[] =
{
	{IO_NULL,IO_NULL},
};

/****************************************************************
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  DAC 的多功能引脚 配置
*Config Info:  GPIO_x (x = A,B,C,D,E,F,G):代表GPIO组,  AFIO_PIN_X(0~15)：代表PIN脚号
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
const afio_init_t dac_afio[] =
{
	{IO_NULL,IO_NULL},
};

const afio_init_t gpio0_afio[]=
{
	{IO_NULL,IO_NULL},	
};
const afio_init_t gpio1_afio[]=
{
	{IO_NULL,IO_NULL},	
};
const afio_init_t gpio2_afio[]=
{
	{IO_NULL,IO_NULL},	
};
const afio_init_t gpio3_afio[]=
{
	{IO_NULL,IO_NULL},	
};
const afio_init_t gpio4_afio[]=
{
	{IO_NULL,IO_NULL},	
};
const afio_init_t gpio5_afio[]=
{
	{IO_NULL,IO_NULL},	
};
const afio_init_t gpio6_afio[]=
{
	{IO_NULL,IO_NULL},	
};

