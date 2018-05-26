//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*by liyongyong*/
#include "am_type.h"
#include "hal_amicro_gpio.h"
#include "gpio_init.h"
#include "afio.h"
#include "adc_chan.h"

/*can not modify.  lyy*/ 
#pragma location = ".CFGINFO"
const afio_t afio_table[] ={ ROBOT_AFIODEF(AM380S_AFIO_CFG_INIT) };/*afio多功能配置表*/

#pragma location = ".CFGINFO"
const IO_PIN_CFG io_table[HAL_MAX] = { ROBOT_IODEF(AM380S_IO_CFG_INIT) };/*提供给sdk用的gpio映射表配置*/

#pragma location = ".CFGINFO"
const adc_chan_t adc_chan_table[ADC_CHAN_MAX] = { ROBOT_ADCCHANDEF(AM380S_ADC_CHAN_INIT) };/*adc 逻辑通道和物理通道配置表*/

#pragma location = ".CFGINFO"
const IO_PIN_INIT_CFG io_init_table[]={ AMICRO_R10_IODEF(AMCIRO_R10_CFG_INST) };/*所有gpio初始化状态表*/

