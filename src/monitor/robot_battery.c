//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include <charge/charge.h>
#include "syscall_api.h"
#include "robot_batter.h"
#include "util/current.h"


static void _battery_update(u32 value);
static void _charge_update(void);

static BatteryStatus _battery_status =
{
    .voltage =  CHARGING_MAXVOL,        //BATTERY_VOLTAGE_FULL,
    .capacity = CHARGING_MAXVOL,        //BATTERY_CAPACITY,   
    .remaining_charge = CHARGING_MAXVOL,       //remaining charge
    .temperature = 30,                         // battery temperature
    .charger_voltage = 0,                       // charger voltage
    .charging_current = 0,
    .charging_state = 0,       // charging state
    .last_status    = 0,

};

#define CHARGER_PULSE_TIME_MIN        2 //ms
#define CHARGER_PULSE_HOLD_TIME       4000 //ms
#define CHARGER_RELASE_TIME           5000 //ms

typedef struct
{
  struct
  {
    U16 holdCount;  
    U8  first;
  } pulse;
  struct
  {
    U8 active;  
    U8 relase;
    U8 last_status;
  } stable;  
} CHARGER_DETECT;

CHARGER_DETECT chargerDetect;

#define BATTERY_STATUS_CHECKJACKIN_INTERVAL     1
#define BATTERY_STATUS_UPDATE_INTERVAL          1000 

/****************************************************************
*Function   :  robot_battery_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  初始化charge状态初始化
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_battery_init(void)
{
    chargerDetect.stable.active   = 0;
    chargerDetect.stable.last_status = CHARGE_NULL;
    chargerDetect.pulse.first = 0;
    chargerDetect.pulse.holdCount = 0;
    timer_task_register(_charge_update,     BATTERY_STATUS_CHECKJACKIN_INTERVAL,    TRUE);
}

/****************************************************************
*Function   :  _charge_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  检测是在dock上，还是在jack上
*CallBy     :  
*Input      :  参数
*              无
*Output     :  chargerDetect.stable.last_status
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
static void _charge_update(void)
{
  static U32 carp_time = 0;
  static U32 relase_time = 0;
  U8  chargerLowActive  = charging_dock();
  U8  chargerHighActive = charging_jack();

  if (chargerHighActive || chargerLowActive)
  {
    chargerDetect.stable.relase = 0;
    if(!chargerDetect.pulse.first)
    {
        carp_time = timer_ms();
    }  
    if(! chargerDetect.stable.active)
    {
      if(timer_elapsed(carp_time) > 10)//10ms
      {
        chargerDetect.stable.active = 1;
      }
    } 
    else if (!chargerDetect.stable.last_status)
    {
      if(chargerLowActive )
        chargerDetect.stable.last_status = DOCKING;
      
      if(chargerHighActive)
        chargerDetect.stable.last_status = JACKING;
    }
    else
    {
      ;
    }
    chargerDetect.pulse.first = 1;
  }
  else
  {
    chargerDetect.stable.active   = 0;
    carp_time = timer_ms();
    
    if(chargerDetect.pulse.first)
    {
      chargerDetect.stable.relase = 1;
      chargerDetect.pulse.first   = 0;
      relase_time = timer_ms();
    }
    
    if(chargerDetect.stable.relase)
    {
      if(chargerDetect.stable.last_status != CHARGE_NULL)
      {
        if( timer_elapsed(relase_time) > CHARGER_RELASE_TIME )
        {
          chargerDetect.stable.last_status = CHARGE_NULL;
        }
      }
    }
  }
}

void robot_battery_monitor_routine(void)
{
  _battery_status.voltage = battery_voltage_average();
  _battery_status.charging_state = charging_detect();
  _battery_status.last_status = chargerDetect.stable.last_status;
  _battery_update(_battery_status.voltage);
  robot_docking_kidnap_monitor(&_battery_status);
  robot_battery_monitor();
}

static u32 _battery_voltage = 3628;
static BOOLEAN _batt_filter_init = 50;;
static u8 _battery_low_continue = 0;
static BOOLEAN _battery_low_status;

static void _battery_update(u32 value)
{
    u32 voltage = value;
    {
        if(!_batt_filter_init)
        {
            _battery_voltage = (((value * 3) + (_battery_voltage * 7))/10);
          
            if(_battery_voltage < BATTERY_LOW_SLOWVECTORY)
            {
              _battery_low_continue++;
              if(_battery_low_continue > 20)//2s
              {
                _battery_low_continue = 0;
                _battery_low_status = 1;
              }              
            }
            else
            {
              if(_battery_low_continue)
                _battery_low_continue--;
              if(!_battery_low_continue)
                 _battery_low_status = 0;
            }
        }
        else
        {
            if(!--_batt_filter_init)
            {
                _battery_voltage = voltage;
            }
        }
    }
}

/****************************************************************
*Function   :  charging_detect
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  判断是否接触上充电器或者充电座
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  1： 接触上  0：非接触上
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 charging_detect(void)
{
   return ( charging_jack() || charging_dock() );
}


BOOLEAN battery_is_charger_present(void)
{
  return charging_detect();
}

BOOLEAN battery_is_dock_present(void)
{
   return charging_detect();
}

u8 is_charging_for_jack(void)
{
  if(charging_detect())
  {
    return (chargerDetect.stable.last_status == JACKING);
  }
  return 0;
}

/****************************************************************
*Function   :  battery_is_low
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  判断是否低电减速
*CallBy     :  提供给sdk使用
*Input      :  无
*Output     :  无
*Return     :  _battery_low_status:  >0 减速   0：正常速度
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 battery_is_low(void)
{
   return _battery_low_status;
}

/****************************************************************
*Function   :  battery_switch_check
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  检测电源开关是否打开
*CallBy     :  无
*Input      :  无
*Output     :  无
*Return     :  1 ：打开   0：关闭
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 battery_switch_check(void)
{
  return (!(gpio_get_value( AM_IO_SWITCH_DET)));
}