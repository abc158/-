//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*by liyongyong*/
#include "am_spi_cfg.h"
#include "syscall_api.h"
#include "am_device.h"

/*spi nor 速度8M*/
#define SPI_NOR_SPEED  (72000000/8000000)

int SPI_NOR_ID  = -1;
int SPI_GYRO_ID = -1;

/****************************************************************
*Function   :  sys_spi_cfg
*Author     :  lyy
*Date       :  2017.4.20
*Description:  spi初始化
*CallBy     :  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sys_spi_cfg(void)
{
    UserSpiInit_t  SPI_InitStructure;
    SPI_NOR_ID = open(DEV_SPI,SPI_CH0);//spi0 
		
    SPI_InitStructure.SpiInit.SPI_Mode = SPI_MASTER;             
    SPI_InitStructure.SpiInit.SPI_FIFO = SPI_FIFO_DISABLE;//SPI_FIFO_ENABLE;//       
    SPI_InitStructure.SpiInit.SPI_DataLength  =  SPI_DATALENGTH_8;      
    SPI_InitStructure.SpiInit.SPI_SELMode     =  SPI_SEL_SOFTWARE;//SPI_SEL_HARDWARE;       
    SPI_InitStructure.SpiInit.SPI_SELPolarity =  SPI_SELPOLARITY_LOW;
    SPI_InitStructure.SpiInit.SPI_CPOL        =  SPI_CPOL_LOW;//SPI_CPOL_HIGH;             
    SPI_InitStructure.SpiInit.SPI_CPHA        =  SPI_CPHA_FIRST;//SPI_CPHA_SECOND;    
    SPI_InitStructure.SpiInit.SPI_FirstBit    =  SPI_FIRSTBIT_MSB;     
    SPI_InitStructure.SpiInit.SPI_RxFIFOTriggerLevel = 1;
    SPI_InitStructure.SpiInit.SPI_TxFIFOTriggerLevel = 4;
    SPI_InitStructure.SpiInit.SPI_ClockPrescaler     = SPI_NOR_SPEED*3/2;  //36   1M   //20 SPI_NOR_SPEED*2/5;
    SPI_InitStructure.irqMode = 0;
    SPI_InitStructure.DmaMode = 0;
    ioctl(SPI_NOR_ID,SPI_INIT,&SPI_InitStructure);
    ioctl(SPI_NOR_ID,SPI_SET_NOR_ID,&SPI_NOR_ID);
    //////////////////7001////////////////////////
    SPI_GYRO_ID = open(DEV_SPI,SPI_CH1); //spi1，目前陀螺仪使用这一路
    SPI_InitStructure.SpiInit.SPI_Mode = SPI_MASTER;             
    SPI_InitStructure.SpiInit.SPI_FIFO = SPI_FIFO_DISABLE;//SPI_FIFO_ENABLE;//       
    SPI_InitStructure.SpiInit.SPI_DataLength  =  SPI_DATALENGTH_8;      
    SPI_InitStructure.SpiInit.SPI_SELMode     =  SPI_SEL_SOFTWARE;//SPI_SEL_HARDWARE;       
    SPI_InitStructure.SpiInit.SPI_SELPolarity =  SPI_SELPOLARITY_LOW;
    SPI_InitStructure.SpiInit.SPI_CPOL        =  SPI_CPOL_LOW;//SPI_CPOL_HIGH;             
    SPI_InitStructure.SpiInit.SPI_CPHA        =  SPI_CPHA_FIRST;//SPI_CPHA_SECOND;    
    SPI_InitStructure.SpiInit.SPI_FirstBit    =  SPI_FIRSTBIT_MSB;     
    SPI_InitStructure.SpiInit.SPI_RxFIFOTriggerLevel = 1;
    SPI_InitStructure.SpiInit.SPI_TxFIFOTriggerLevel = 4;
    SPI_InitStructure.SpiInit.SPI_ClockPrescaler     = SPI_NOR_SPEED;  //36   1M
    SPI_InitStructure.irqMode = 0;
    SPI_InitStructure.DmaMode = 0;
    
    ioctl(SPI_GYRO_ID,SPI_INIT,&SPI_InitStructure); 
    ioctl(SPI_GYRO_ID,SPI_SET_GYRO_ID,&SPI_GYRO_ID);//配置陀螺仪ID号
}

void sys_spi_close(void)
{
   close(SPI_NOR_ID);
   close(SPI_GYRO_ID);  
   SPI_NOR_ID = -1;
   SPI_GYRO_ID =-1;
}

int spi0_id_get(void)
{
  return SPI_NOR_ID;
}

int spi1_id_get(void)
{
  return SPI_GYRO_ID;
}

