#ifndef __SYS_CFG_H
#define __SYS_CFG_H
#include "DateBase.h"
typedef struct
{
  U32 verson;
  power_info    power;
  hard_info     hard;  
  sensor_info   sensor;
  act_info      act;
  ir_info       remote;
  gpio_config_t gpio;
}SYS_CONFIG;

#endif