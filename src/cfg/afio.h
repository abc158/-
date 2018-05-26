#ifndef __AFIO_H__
#define __AFIO_H__
#include "am_gpio.h"

#define USER_AFIO_MAX 8

typedef enum
{
  MOD_ADC  ,	
  MOD_SPI0 ,
  MOD_SPI1 ,
  MOD_UART0,
  MOD_UART1,
  MOD_UART2,
  MOD_UART3,
  MOD_I2C0 ,
  MOD_I2C1 ,
  MOD_PID  ,  
  MOD_MCTM0,
  MOD_MCTM1,
  MOD_GPTM0,
  MOD_GPTM1,
  MOD_GPTM2,
  MOD_IR   ,
  MOD_LED  ,  
  MOD_DAC  ,
  MOD_GA   ,
  MOD_GB   ,
  MOD_GC   ,
  MOD_GD   ,
  MOD_GE   ,
  MOD_GF   ,
  MOD_GG   ,
  MOD_MAX  ,
  MOD_NULL = 0xff,
}MOD_E;

typedef enum
{
  GPIO_A = 0,  /*!< EXTI channel x source come from GPIO Port A */
  GPIO_B,      /*!< EXTI channel x source come from GPIO Port B */
  GPIO_C,      /*!< EXTI channel x source come from GPIO Port C */
  GPIO_D,      /*!< EXTI channel x source come from GPIO Port D */
  GPIO_E,      /*!< EXTI channel x source come from GPIO Port E */
  GPIO_F,      /*!< EXTI channel x source come from GPIO Port F */
  GPIO_G,      /*!< EXTI channel x source come from GPIO Port G */  
  IO_NULL = 0xff
}GPIOX_E;

typedef struct
{
  GPIOX_E   Gpiox;
  uint32_t  Afiox;
}afio_init_t;


typedef struct
{
  MOD_E           mod;
  AFIO_MODE_Enum  mode;	
  const afio_init_t  *afio;
}afio_t;


extern const afio_init_t adc_afio[];  
extern const afio_init_t spi0_afio[]; 
extern const afio_init_t spi1_afio[]; 
extern const afio_init_t uart0_afio[];
extern const afio_init_t uart1_afio[];
extern const afio_init_t uart2_afio[];
extern const afio_init_t uart3_afio[];
extern const afio_init_t i2c0_afio[]; 
extern const afio_init_t i2c1_afio[]; 
extern const afio_init_t pid_afio[];  
extern const afio_init_t mctm0_afio[];
extern const afio_init_t mctm1_afio[];
extern const afio_init_t gptm0_afio[];
extern const afio_init_t gptm1_afio[];
extern const afio_init_t gptm2_afio[];
extern const afio_init_t ir_afio[];   
extern const afio_init_t led_afio[];  
extern const afio_init_t dac_afio[];  
extern const afio_init_t gpio0_afio[];
extern const afio_init_t gpio1_afio[];
extern const afio_init_t gpio2_afio[];
extern const afio_init_t gpio3_afio[];
extern const afio_init_t gpio4_afio[];
extern const afio_init_t gpio5_afio[];
extern const afio_init_t gpio6_afio[];

#define AM380S_AFIO_CFG_INIT(name,mode,table) \
  {name,mode,table} ,

/*以下表格不能更改，只需要修改上面表格即可，lyy*/
#define ROBOT_AFIODEF(m)      \
  /***************table head*************/\
  m(0x55  ,0xaa ,0)\
  /*************************************/    \
  m(MOD_ADC  ,AFIO_MODE_2  ,&adc_afio[0]  )\
  m(MOD_SPI0 ,AFIO_MODE_3  ,&spi0_afio[0] )\
  m(MOD_SPI1 ,AFIO_MODE_3  ,&spi1_afio[0] )\
  m(MOD_UART0,AFIO_MODE_4  ,&uart0_afio[0])\
  m(MOD_UART1,AFIO_MODE_4  ,&uart1_afio[0])\
  m(MOD_UART2,AFIO_MODE_4  ,&uart2_afio[0])\
  m(MOD_UART3,AFIO_MODE_4  ,&uart3_afio[0])\
  m(MOD_I2C0 ,AFIO_MODE_5  ,&i2c0_afio[0] )\
  m(MOD_I2C1 ,AFIO_MODE_5  ,&i2c1_afio[0] )\
  m(MOD_PID  ,AFIO_MODE_6  ,&pid_afio[0]  )\
  m(MOD_MCTM0,AFIO_MODE_7  ,&mctm0_afio[0])\
  m(MOD_MCTM1,AFIO_MODE_7  ,&mctm1_afio[0])\
  m(MOD_GPTM0,AFIO_MODE_7  ,&gptm0_afio[0])\
  m(MOD_GPTM1,AFIO_MODE_7  ,&gptm1_afio[0])\
  m(MOD_GPTM2,AFIO_MODE_7  ,&gptm2_afio[0])\
  m(MOD_IR   ,AFIO_MODE_8  ,&ir_afio[0]   )\
  m(MOD_LED  ,AFIO_MODE_9  ,&led_afio[0]  )\
  m(MOD_DAC  ,AFIO_MODE_11 ,&dac_afio[0]  )\
  m(MOD_GA   ,AFIO_MODE_2  ,&gpio0_afio[0])\
  m(MOD_GB   ,AFIO_MODE_2  ,&gpio1_afio[0])\
  m(MOD_GC   ,AFIO_MODE_2  ,&gpio2_afio[0])\
  m(MOD_GD   ,AFIO_MODE_2  ,&gpio3_afio[0])\
  m(MOD_GE   ,AFIO_MODE_2  ,&gpio4_afio[0])\
  m(MOD_GF   ,AFIO_MODE_2  ,&gpio5_afio[0])\
  m(MOD_GG   ,AFIO_MODE_2  ,&gpio6_afio[0])\
  m(MOD_MAX  ,0,0)


#endif