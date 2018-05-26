#ifndef AM_EXTI_H
#define AM_EXTI_H
#include "am_type.h"

#define EXTI_WAKEUP_HIGH_LEVEL                      ((u8)0x0)
#define EXTI_WAKEUP_LOW_LEVEL                       ((u8)0x1)

typedef enum
{
  EXTI_LOW_LEVEL     = 0x0,
  EXTI_HIGH_LEVEL    = 0x1,
  EXTI_NEGATIVE_EDGE = 0x2,
  EXTI_POSITIVE_EDGE = 0x3,
  EXTI_BOTH_EDGE     = 0x4
}EXTIInterrupt_TypeDef;

typedef struct
{
  u8 IntEn;       //enable irq
  u8 Int_type;    //low/hight/n/h/both
  u8 WakeupirqEn; //wakeup irq en
  u8 Wakeup_type; //wakeup l/h event
}exti_init_t;

s16 exit_wakeup_cfg(void);
s16 wakeup_pin_cfg(void);

#endif