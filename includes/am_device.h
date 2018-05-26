#ifndef _DEVICE_H_
#define _DEVICE_H_

//系统的硬件模块,通过下边设备来操作,设备的就当成是文件设备概念。
typedef enum
{
  DEV_UART = 0, /*串口 设备号*/
  DEV_BFTM ,    /*base timer 设备号,目前暂无开放给app使用，因为系统使用完了*/
  DEV_GPTM ,    /*通用 timer 设备号*/
  DEV_IR   ,    /*IC 自带的IR 设备号*/
  DEV_EXTI ,    /*外部中断 设备号*/
  DEV_RTC  ,    /*rtc 设备号，暂时不通过设备访问，直接通过api接口*/
  DEV_WDG  ,    /*watchdog 设备号*/
  DEV_BAKUP,    /*备份模块 设备号，2个空余的32位可用，关闭电源开关后数据依然不丢，这个可以用来存放一个标志等等*/
  DEV_SPI  ,    /*SPI 设备号*/
  DEV_I2C  ,    /*i2c 设备号*/
  DEV_IPC  ,    /*ipc 设备号， 暂时没有使用*/
  DEV_MCTM ,    /*多功能 timer 设备号*/
}DEVICE_E;

//IPC通信的类型
typedef enum
{
  IPC_USER  =1,
  IPC_KERNEL=2,
}IPC_TYPE_E;

/* IC 总共最多支持4路的串口， 每个串口id号*/
typedef enum
{
        UART_NULL  = 0,
	USART_ID0 = 1,
	USART_ID1 = 2,
	UART_ID0  = 3,
	UART_ID1  = 4
}UART_ID_E;

//驱动模块的命令字
#define SYS_TM_START    0x10000  
#define SYS_UART_START  0x20000
#define SYS_WDT_START   0x30000
#define SYS_SPI_START   0x40000
#define SYS_RTC_START   0x50000
#define SYS_BAKUP_START 0x60000
#define SYS_I2C_START   0x70000
#define SYS_IPC_START   0x80000
#define SYS_EXTI_START  0x90000
#define SYS_IR_START    0xa0000

//rtc mod
typedef enum
{
	RTC_INIT      = SYS_RTC_START + 0, 
	RTC_EXIT      = SYS_RTC_START + 1
}SYS_RTC_CMD_E;

//spi mod
typedef enum
{
	SPI_INIT      = SYS_SPI_START + 0,   /*初始化spi，ioctl参数作为input， 为 UserSpiInit_t 类型*/
	SPI_EXIT      = SYS_SPI_START + 1,   /*关闭spi， ioctl 参数 没有使用参数*/
	SPI_CS        = SYS_SPI_START + 2,   /*控制spi cs线的高低，ioctl参数作为input，为U8类型*/
	SPI_FIFO_RX_STATE= SYS_SPI_START + 3,/*获取spi rx 硬件数据缓冲区是否有数据，ioctl参数作为output，为U8类型*/
        SPI_FIFO_TX_STATE= SYS_SPI_START + 9,/*获取spi tx 硬件数据缓冲区是否满，ioctl参数作为output，为U8类型*/
	SPI_BUSY      = SYS_SPI_START + 4,   /*获取spi总线是否busy状态，ioctl参数，没有使用参数*/
	SPI_START     = SYS_SPI_START + 5,   /*启动spi 传送*/
	SPI_STOP      = SYS_SPI_START + 6,   /*停止spi 传送*/
        SPI_SET_GYRO_ID = SYS_SPI_START + 7, /*配置陀螺仪的SPI ID号，ioctl参数作为input，为int型*/
        SPI_SET_NOR_ID  = SYS_SPI_START + 8, /*配置nor 的SPI ID号，ioctl参数作为input，为int型*/       
	
}SYS_SPI_CMD_E;

//wdg mod
typedef enum
{
	WDT_INIT      = SYS_WDT_START + 0,/*wdt,初始化，目前暂时不需要*/
	WDT_START     = SYS_WDT_START + 1,/*wdt, 使能watchdog功能， ioctl参数作为input，为u16类型，是单位是ms*/
	WDT_STOP      = SYS_WDT_START + 2,/*停止wdt， ioctl 参数 没有使用参数*/
	WDT_FEED      = SYS_WDT_START + 3,/*喂狗，    ioctl 参数 没有使用参数*/
}SYS_WDT_CMD_E;

//uart mod
typedef enum
{
	UART_INIT      = SYS_UART_START + 0,  /*uart初始化，ioctl参数作为input，为UserUartInit_t类型*/
	UART_EXIT      = SYS_UART_START + 1,  /*关闭uart*/
	UART_FIFO_FULL = SYS_UART_START + 2,  /*uart 判断硬件缓冲区是否满，ioctl参数作为input，为U8类型*/
	UART_FIFO_EMPTY= SYS_UART_START + 3,  /*uart 判断硬件缓冲区是否空，ioctl参数作为input，为U8类型*/
        UART_TRIG_TX   = SYS_UART_START + 4,  /*uart 触发一个tx中断*/
        UART_SET_CONSOLE = SYS_UART_START + 5,/*uart 设置打印口控制台的uart id号*/
        UART_GET_CONSOLE = SYS_UART_START + 6,/*uart 获取打印口控制台的uart id号*/
}SYS_UART_CMD_E;

//tm mod
typedef enum
{
	GPTM_CHAN0_SET_DUTY  = SYS_TM_START + 1, /*通用timer， 0通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	GPTM_CHAN0_GET_DUTY  = SYS_TM_START + 2, /*通用timer， 0通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/
	GPTM_CHAN1_SET_DUTY  = SYS_TM_START + 9, /*通用timer， 1通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	GPTM_CHAN1_GET_DUTY  = SYS_TM_START + 10,/*通用timer， 1通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/	
	GPTM_CHAN2_SET_DUTY  = SYS_TM_START + 11,/*通用timer， 2通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	GPTM_CHAN2_GET_DUTY  = SYS_TM_START + 12,/*通用timer， 2通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/
	GPTM_CHAN3_SET_DUTY  = SYS_TM_START + 13,/*通用timer， 3通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	GPTM_CHAN3_GET_DUTY  = SYS_TM_START + 14,/*通用timer， 3通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/
        
	MCTM_CHAN0_SET_DUTY  = SYS_TM_START + 15,/*mc timer， 0通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	MCTM_CHAN0_GET_DUTY  = SYS_TM_START + 16,/*mc timer， 0通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/
	MCTM_CHAN1_SET_DUTY  = SYS_TM_START + 17,/*mc timer， 1通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	MCTM_CHAN1_GET_DUTY  = SYS_TM_START + 18,/*mc timer， 1通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/	
	MCTM_CHAN2_SET_DUTY  = SYS_TM_START + 19,/*mc timer， 2通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	MCTM_CHAN2_GET_DUTY  = SYS_TM_START + 20,/*mc timer， 2通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/
	MCTM_CHAN3_SET_DUTY  = SYS_TM_START + 21,/*mc timer， 3通道设置pwm占空比，ioctl参数为input，为uint16_t类型*/
	MCTM_CHAN3_GET_DUTY  = SYS_TM_START + 22,/*mc timer， 3通道获取pwm占空比，ioctl参数为input，为uint16_t类型*/        
        
	TM_BFTM_INIT = SYS_TM_START + 3,/*base 目前暂时没有给app工程使用*/
	TM_BFTM_EXIT = SYS_TM_START + 4,/*base 目前暂时没有给app工程使用*/
        
	TM_GPTM_BASE_INIT = SYS_TM_START + 5,     /*通用 timer， 初始化，ioctl参数为input，为 GPTM_TimeBaseInitTypeDef 类型*/
	TM_GPTM_OUT_MODE_INIT = SYS_TM_START + 6, /*通用 timer， 初始化，ioctl参数为input，为 UserOutInit_t 类型*/
	TM_GPTM_IN_MODE_INIT  = SYS_TM_START + 7, /*通用 timer， 初始化，ioctl参数为input，为 UserCapInit_t 类型*/
        TM_GPTM_EXIT = SYS_TM_START + 8,          /*通用 timer， 初始化，ioctl参数，没有使用*/    
        
	TM_MCTM_BASE_INIT = SYS_TM_START + 23,    /*mc timer， 初始化，ioctl参数为input，为 MCTM_TimeBaseInitTypeDef 类型*/
	TM_MCTM_OUT_MODE_INIT = SYS_TM_START + 25,/*mc timer， 初始化，ioctl参数为input，为 MUserOutInit_t 类型*/
	TM_MCTM_IN_MODE_INIT  = SYS_TM_START + 26,/*mc timer， 初始化，ioctl参数为input，为 MUserCapInit_t 类型*/
        TM_MCTM_EXIT = SYS_TM_START + 27,         /*mc timer， 初始化，ioctl参数，没有使用*/     
        
}SYS_TM_CMD_E;

typedef enum
{
  BAKUP_INIT   = SYS_BAKUP_START + 0,
  BAKUP_EXIT   = SYS_BAKUP_START + 1,
}SYS_BAKUP_CMD_E;

typedef enum
{
  EXTI_INIT   = SYS_EXTI_START + 0,/*外部中断初始化, ioctl 参数为input，为 exti_init_t 类型*/
  EXTI_EXIT   = SYS_EXTI_START + 1,/*外部中断quit*/
}SYS_EXTI_CMD_E;

typedef enum
{
	IPC_INIT       = SYS_IPC_START + 0,
	IPC_EXIT       = SYS_IPC_START + 1,
	IPC_FIFO_EMPTY = SYS_IPC_START + 2,
}SYS_IPC_CMD_E;

#endif