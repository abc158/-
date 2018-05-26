//----------------------------------------------------------------------------
//  Copyright (C) 2010, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.
//----------------------------------------------------------------------------
#include <charge/charge.h>
#include <ui-config.h>


static am_uint8  _battery_remaining_percentage;
static am_uint16 _battery_low_continue;
static am_uint16 _battery_cut_continue;
static am_uint8  _battery_cut_cnt;
static uint16_t battery_voltage_dock = BATTERY_CHARGING_LOW_VOLTAGE;
static uint16_t battery_voltage_shutoff = BATTERY_CHARGING_CUTOFF_VOLTAGE;
extern u8 g_clean_resume;
extern AM_Pose2D resume_pose;
extern const sensor_frame_t* rq_sensor_data;
extern am_uint8 am_dock_get_raw_beacons(robot_dock_signals_t *am_dock_signals);
void sys_shutdown(void);
extern am_uint8 get_clean_start(void);


static void _robot_docking_kidnap_monitor(BatteryStatus *battery_status)
{
    static DockingKidnap prev_status;
    static uint8_t debounce_none = 0;
    static uint8_t debounce_ing = 0;

    static DockingKidnap cur_status;

    if( (IS_RUNNING_MODE(get_main_mode())) && (get_main_mode()!=DOCK) )
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

    if( (prev_status == DOCK_ING) && (cur_status == DOCK_NONE) && ((get_main_mode() == IDLE)|| (get_main_mode() == SLEEP)) )
    {
      cur_status = DOCK_KIDNAP;
      if( !IS_RUNNING_MODE(get_main_mode()) && (get_main_mode()!=FACTORY_TEST) )
      {
        AM_DEBUG_KEY("dock kidnap \r\n");
        uint8_t ir_signal_ok = 0;
        //robot_dock_signals_t dock_signals;
        //am_dock_get_raw_beacons(&dock_signals);
#if 0 //not to us
        if( (ir_signal == 1) && (timer_elapsed(ir_time)<3000))
        {
          AM_DEBUG_KEY("ok\r\n");
          ir_signal    = 0;
          ir_signal_ok = 1;
        }
#endif
        if( (battery_status->last_status == DOCKING) || ir_signal_ok )
        {
            if(robot_wheel_is_dropped(AM_LEFT) && robot_wheel_is_dropped(AM_RIGHT))
            {
              AM_DEBUG_KEY("w drop\r\n");
            }
            else
            {
              event_send(&app_ctrl_event_queue, AC_E_DOCK, EVENT_DOCK_KIDNAP);//set the state to idle from init
            }
        }
        else
        {
            AM_DEBUG_KEY("sig=%d,%d\r\n",ir_signal,battery_status->last_status);
        }
        
      }
    }
    prev_status = cur_status;    
}

// Translates the battery status changes to application events.
//10ms call it
extern U8 get_fullgo_mode(void);
extern U8 set_fullgo_mode(u8 flag);
static void _robot_battery_monitor(BatteryStatus *battery_status)
{
#if(1) //old-mask  need to do it, check the battery status
    _battery_remaining_percentage = (am_uint8)(((am_uint32)battery_status->voltage * 100) / battery_status->capacity);
      
    if(battery_status->charging_state)
    {
      _battery_low_continue = 0;
      _battery_cut_continue = 0;
      _battery_cut_cnt = 0;
    }
    else
    {
      if(battery_status->voltage <= battery_voltage_shutoff)
      {
         _battery_cut_continue++;
         if(_battery_cut_continue > 5*100)//5S
         {
             if(!charging_detect())
             {
               AM_DEBUG_ERR("bvl,shutdown\r\n");
               if(_battery_cut_cnt++ >= 4)
               {
                 sys_shutdown(); 
               }
               _battery_cut_continue=0;
               event_send(&app_ctrl_event_queue, AC_N_BATT_CRITICAL, EVENT_ARG_DEFAULT);
             }
         }
      }
      else if(battery_status->voltage <= battery_voltage_dock)
      {
          _battery_low_continue++;
          if(_battery_low_continue > 10*100)//15S
          {
            AM_DEBUG_ERR("bat is low,go home...\r\n");
            _battery_low_continue = 0;
            if( ( get_main_mode()==CLEAN_ROOM ) || ( get_main_mode()== CLEAN_AUTO ) )
            {
#ifdef LONGTEST_MODE
              set_long_run_test_mode(0);
#else
#ifdef LONGTEST_DOCK_MODE 
              set_long_run_test_mode(0);
#endif
#endif
              if(get_fullgo_mode()) //fullgo mode ,  if bat low,  clear 
              {
                  set_fullgo_mode(0);
              }
              
              if(get_clean_resume_en())  //if support resume lyy
              {
                set_clean_resume_state(1);
                memcpy(&resume_pose,mqs_get_robot_pose(),sizeof(AM_Pose2D));
              }
              event_send(&app_ctrl_event_queue, AC_E_DOCK, EVENT_ARG_DEFAULT);//set the state to idle from init
            }
          }
      }
      else
      {
        if(_battery_cut_continue)
          _battery_cut_continue--;
        
        if(_battery_low_continue)
          _battery_low_continue--;
        
        if(_battery_cut_cnt)
          _battery_cut_cnt--;
      }
    }
    _prev_battery_status = *battery_status;
#endif
}
//10ms call it
static void _robot_power_monitor_status_callback(BatteryStatus *battery_status)
{
  _robot_battery_monitor(battery_status);
  _robot_docking_kidnap_monitor(battery_status);
}

void power_config_update(ui_config_t* config)
{
  if(config->power_config.battery_voltage_min_adc)//回座子最低电压
  {
      battery_voltage_dock = config->power_config.battery_voltage_min_adc;
  }
  
  if(config->power_config.battery_voltage_dead_adc)//关闭DCDC最低电压
  {
      battery_voltage_shutoff = config->power_config.battery_voltage_dead_adc;
  }
}


void robot_battery_monitor_init(void)
{
    battery_set_callback(_robot_power_monitor_status_callback);
    //ir_signalCallback(_robot_ir_signal);
    //ui_interface_register_config_update_callback(power_config_update);
}

