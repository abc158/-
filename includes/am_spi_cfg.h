#ifndef AM_SPI_CFG_H
#define AM_SPI_CFG_H
#include "AM38x_spi.h"

typedef u32 (*spi_fun_ptr)(u32 , u32);

typedef enum
{
  SPI_NULL = 0,
  SPI_CH0 = 1,
  SPI_CH1 = 2,
}SPI_CH_E;

typedef struct
{
  u32                   irqMode;
  u32                   DmaMode;
  SPI_InitTypeDef       SpiInit;
}UserSpiInit_t;

extern void sys_spi_cfg(void);
extern void sys_spi_close(void);
extern int  spi0_id_get(void);
extern int  spi1_id_get(void);
#endif