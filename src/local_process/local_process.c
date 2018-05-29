//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include <ui-config.h>
#include "ui-commands.h"
#include "am_date_base.h"
#include "time.h"
#include  <am_config_macro.h>
#include "am_key.h"
#include "act.h"
#include "local_key_check.h"
#include "remote.h"
#include "display.h"
#include "ui-song-player.h"
#include "syscall_api.h"
#include "wireless/ARF2496K.H"
#include "sensor/sensor.h"
#include "motor/motor.h"
#include "monitor/robot_batter.h"
#include "ui-manager/exception.h"
#include "am_device.h"
#if defined(USE_WIFI_DEMO_1)
#include "wifi/wifi_demo1/simwifi.h"
#include "wifi/wifi_demo1/simsweep.h"
#elif defined(USE_WIFI_DEMO_2)
#include "wifi/wifi_demo2/simwifi_demo2.h"
#include "wifi/wifi_demo2/SimSweep_demo2.h"
#endif
#define USE_WATCHDOG_MONITOR  //需要打开看门狗监控，则需要定义
#define FEED_DOG_MAX_TIME     10000 //ms

/* 唤醒源 */
volatile U8 wakeupSource;
/*系统空闲计时*/
U16 idle_cnt;
int wdg_fd =-1;

extern void ir_rx_capture_4khz(void);
extern void ui_rx_server( void);
extern void ui_uart_cmd_rounte(void);
extern void robot_current_init(void);
extern void set_adjust_adc_value_to_system(void);
extern void wifi_uart_cmd_rounte(void);
extern u8 battery_switch_check(void);
/****************************************************************
*Function   :  start_watchdog
*Description:  启动看门狗 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void start_watchdog()
{
#ifdef USE_WATCHDOG_MONITOR
  u16 max_check_time = FEED_DOG_MAX_TIME;//ms, 10s
  wdg_fd = open(DEV_WDG,0);
  ioctl(wdg_fd,WDT_START,&max_check_time);  
#endif
}

/****************************************************************
*Function   :  stop_watchdog
*Description:  停止看门狗 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void stop_watchdog(void)
{
#ifdef USE_WATCHDOG_MONITOR  
  ioctl(wdg_fd,WDT_STOP,0); 
#endif
}

/****************************************************************
*Function   :  feed_watchdog
*Description:  喂狗 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void feed_watchdog(void)
{
#ifdef USE_WATCHDOG_MONITOR  
  ioctl(wdg_fd,WDT_FEED,0); 
#endif
}

/****************************************************************
*Function   :  local_app_init
*Description:  app初始化  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void local_app_init(void)
{
  /*init all app*/
  _key_init();
  _remote_init();
  _act_init();
  _display_init();
  set_display_power_up_flag(1);
  songplayer_init();
  robot_ir_detection_init();
  robot_motor_init();
  robot_battery_init();
  robot_current_init();
  robot_universl_wheel_init();//初始化，exit需要在睡眠时处理
  set_adjust_adc_value_to_system();
  //InitARF2496k();
  sys_timer_register(HZ_4K,(long)ir_rx_capture_4khz,1);//红外优先级最高

  #if defined(USE_WIFI_DEMO_1)
  UART_Config_PinResetWifiCard();
  #elif defined(USE_WIFI_DEMO_2)
  wifi_init();
  #endif
  if(battery_switch_check() == 1)
  {
      songplayer_play_id(SONG_ID_POWER_UP, 1);
  }
  start_watchdog();
  
}



/****************************************************************
*Function   :  main_app_task
*Description:  UI的处理函数，每10ms执行一次  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
extern void robot_side_brush_adjust_set(u16 speed);
extern void robot_mid_brush_adjust_set(u16 speed);
extern void robot_suction_adjust_set(u16 val);
extern void robot_suction_ctrl(BOOLEAN en);
extern void robot_sidebrush_vols_set(u16 speed);
extern void robot_midbrush_vols_set(u16 speed);
extern void print_cliff();
extern void print_touch();
extern void print_debug_lt_auto();
void main_app_task(void *arg)
{	  
  UI_STATE_E s;
  while(1)
  {  
  print_touch();
    {
      key_routine();
      remote_routine();
      #if defined(USE_UART_WIFI)
      #if defined(USE_WIFI_DEMO_1)
      WifiData_Period_Process();
      #elif defined(USE_WIFI_DEMO_2)
      uart_server_routine();
      #endif     
      wifi_uart_cmd_rounte();
      #endif
      
    } 

    {      
      s = get_ui_state();
      //if(s != UI_ENTER_SLEEPING)
      {                    
        act_routine(); 
        display_routine(); 
        songplayer_routine();
      } //else {
        //songplayer_quit_playing();
      //}
      robot_battery_monitor_routine();
      sys_exception_monitor();
      if(s == UI_TEST)
      {
        self_test_routine();
      }
      ui_handle_idle(); 
      #if defined(USE_WIFI_DEMO_2)
      map_data_process();
      #endif
    }
    feed_watchdog();
    sleep(1);
  }
  
}

