//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/

#include <charge/charge.h>
#include <ui-config.h>
#include "robot_batter.h"
#include "syscall_api.h"
#include "ui-test_cmd_handle.h"
#include "am_date_base.h"
#include "act.h"
#include "util/current.h"
#include "ui-song-player.h"
#include DOCKING_NEW_HEADER


static u16 _battery_low_continue = 0;
static u16 _battery_cut_continue = 0;
static u8  _battery_cut_cnt;
static u16 battery_voltage_dock    = CHARGING_LOW_VOLTAGE;
static u16 battery_voltage_shutoff = CHARGING_CUTOFF_VOLTAGE;
extern uint32_t dock_signal_get_time;
static u8 dock_kidnap_state = 0;
u8 get_dock_kidnap_state() {
    return dock_kidnap_state;
}
void set_dock_kidnap_state(u8 s) {
    dock_kidnap_state = s;
}

/****************************************************************
*Function   :  robot_docking_kidnap_monitor
*Author     :  lyy
*Date       :  2017.4.20
*Description:  检测机器在充电状态过程中是否被意外移除动
*CallBy     :  10ms call it again
*Input      :  参数
*              battery_status ： 当前的状态
*Output     :  无
*Return     :  无
*Others     :
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_docking_kidnap_monitor(BatteryStatus *battery_status)
{
    static DockingKidnap prev_status;
    static uint8_t debounce_none = 0;
    static uint8_t debounce_ing = 0;

    static DockingKidnap cur_status;

    if( (IS_RUNNING_MODE(sys_runing_mode_get())) && (sys_runing_mode_get()!=ROBOT_STATE_DOCK) )
    {
      debounce_ing = 0;
      debounce_none = 0;
      prev_status =DOCK_NONE;
      cur_status=DOCK_NONE;
      return;
    }

    if(battery_status->charging_state == 0)
    {
        debounce_ing = 0;
        debounce_none ++;
        if(debounce_none > 150)//10
        {
          debounce_none = 0;
          cur_status = DOCK_NONE;
        }
    }
    else
    {
        debounce_none = 0;
        debounce_ing++;
        if(debounce_ing >= 2)//20ms
        {
          debounce_ing = 0;
          cur_status = DOCK_ING;
        }
    }

    if( (prev_status == DOCK_ING) && (cur_status == DOCK_NONE) && \
        ((sys_runing_mode_get() == ROBOT_STATE_WAITING)|| (sys_runing_mode_get() == ROBOT_STATE_SLEEP)) )
    {
      cur_status = DOCK_KIDNAP;
      if( !IS_RUNNING_MODE(sys_runing_mode_get()) && (sys_runing_mode_get()!=ROBOT_STATE_TESTING) )
      {
        printf("dock kidnap \r\n");

        if( (battery_status->last_status == DOCKING) )
        {
            if(robot_wheel_drop_mask()==(WHEEL_DROP_STATE_RIGHT|WHEEL_DROP_STATE_LEFT))
            {
              printf("w drop\r\n");
            }
            else
            {
              //go_to_dock();//need to do ,,,,,...... //to do...
              printf("need to dock\r\n");
              local_map_reset(0);
              if(timer_elapsed(dock_signal_get_time)<250)
              {
                set_start_docking_state(1);
                set_dock_leave_dockagain(1);
								#ifdef AM_DOCKING_METHOD
            	  set_dock_kidnap_state(1);
              	#endif
              }
            }
        }
        else
        {
            printf("sta=%d\r\n",battery_status->last_status);
        }

      }
    }
    prev_status = cur_status;
}

void delayus()
{
	uint16_t i;
	for(i=0;i<0x3000;i++)//3.6us
	{
	   ;
	}
}

/****************************************************************
*Function   :  sys_shutdown
*Description:  关闭DCDC
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :
******************************************************************/
void sys_shutdown(void)
{
  int i=0;

  songplayer_play_id(SONG_ID_LOW_POWER, 0);
  while(songplayer_is_playing())
  {
    sleep(10);
  }

  while(i<0xffff)
  {
    i++;
    gpio_set_value(IO_VCC_KILL, 0);
    delayus();
    gpio_set_value(IO_VCC_KILL, 1);
    delayus();
  }
}

/****************************************************************
*Function   :  robot_docking_kidnap_monitor
*Author     :  lyy
*Date       :  2017.4.20
*Description:  检测机器电压是否过低，或者是否低电回座
*CallBy     :  10ms call it again
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_battery_monitor(void)
{

    if(charging_detect())
    {
      _battery_low_continue = 0;
      _battery_cut_continue = 0;
      _battery_cut_cnt = 0;
    }
    else
    {
      if(battery_voltage_average() <= battery_voltage_shutoff)
      {
         _battery_cut_continue++;
         if(_battery_cut_continue > 5*100)//5S
         {
             if(!charging_detect())
             {
               printf("bvl,shutdown\r\n");
               _battery_cut_continue=0;
               //to do ,,,, go to shutdown system
               sys_shutdown();
             }
         }
      }
      else if(battery_voltage_average() <= battery_voltage_dock)
      {
          _battery_low_continue++;
          if(_battery_low_continue > 10*100)//15S
          {
            _battery_low_continue = 0;
            if( ( sys_runing_mode_get()==ROBOT_STATE_CLEANING_ROOM ))
            {
              //go_to_dock();//need to do ,,,,,
              printf("bat is low,go home...\r\n");
              set_start_docking_state(1);
            }
          }
      }
      else
      {
        //if(_battery_cut_continue)
        //  _battery_cut_continue--;
        _battery_cut_continue = 0;

        if(_battery_low_continue)
          _battery_low_continue--;

        if(_battery_cut_cnt)
          _battery_cut_cnt--;
      }
    }
}

