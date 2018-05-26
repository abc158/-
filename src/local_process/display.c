//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include <ui-config.h>
#include "ui-commands.h"
#include "am_date_base.h"
#include "am_config_macro.h"
#include "time.h"
#include "lib.h"
#include  <am_config_macro.h>
#include "local_key_check.h"
#include "remote.h"
#include "act.h"
#include "display.h"
#include "syscall_api.h"
#include "led_drv.h"
#include "charge/charge.h"

U8 display_power_up_flag = 0;           /*上电显示标志*/
U8 last_flush_state = 0;                /*灯闪烁状态*/
static sys_state_info sys_state_info_p; /*系统状态*/

int led_gpio_group[LED_GPIO_NUM]=
{
  AM_I0_CLEAN_LED  ,
  AM_I0_DOCK_LED   ,
  AM_I0_SPOT_LED    
};

/****************************************************************
*Function   :  ui_handle_sleep
*Description:  发送sys_power后，系统的回调函数  
*Input      :  handle_time
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void ui_handle_sleep(u8 handle_time)
{
  U8 sleep_state = get_enter_sleep_state();
  //if(handle_time == 1)
  {
    //if(sleep_state == SLEEP_MODE_S1)
    {
      //set_led(ALL_LED);
    }
  }
 // wifi_handle_sleep();
  
}

static power_ctrl_t sleep_funcs = {
  .sleep = ui_handle_sleep,
  .wake  = NULL,
};

/****************************************************************
*Function   :  display_gpio_init
*Description:  申请显示需要的IO  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
U8 display_gpio_init(void)
{
  int ret = -1;
  for( int i=0; i<LED_GPIO_NUM; i++ )
  {
    ret = gpio_request_one(led_gpio_group[i],GPIO_F_DIR_OUT|GPIO_F_INIT_LOW);
    
    if(ret != -1)
    {
      ;
    }
    else
    {
      printf("display io not ok\r\n");
    }
  }  
  return 0;
}

/****************************************************************
*Function   :  _display_init
*Description:  显示初始化  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _display_init(void)
{
  set_power_callback(&sleep_funcs);   
  display_gpio_init();
#ifdef USE_SPI_DISPLAY
  led_drv_init(); 
#endif
}

/****************************************************************
*Function   :  _display_exit
*Description:  显示退出  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _display_exit(void)
{
  ;
}

/****************************************************************
*Function   :  set_led
*Description:  点亮某个颜色的灯  
*Input      :  led_id：需要点亮的那个灯
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void set_led(U8 led_id)
{
  switch(led_id)
  {
  case CLEAN_LED:
    {
      gpio_set_value(AM_I0_CLEAN_LED, 1);
    }
    break;
  case DOCK_LED:
    {
      gpio_set_value(AM_I0_DOCK_LED,1);
    }
    break;
  case SPOT_LED:
    {
      gpio_set_value(AM_I0_SPOT_LED,1);
      //gpio_set_value(AM_I0_CLEAN_LED,1);
    }
    break;
  case ALL_LED:
    {
      gpio_set_value(AM_I0_CLEAN_LED,1);
      gpio_set_value(AM_I0_DOCK_LED,1);
      gpio_set_value(AM_I0_SPOT_LED,1);
    }     
    break;
  }
}


/****************************************************************
*Function   :  reset_led
*Description:  熄灭某个颜色的灯  
*Input      :  led_id：需要熄灭的那个灯
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void reset_led(U8 led_id)
{
  switch(led_id)
  {
  case CLEAN_LED:
    {
      gpio_set_value(AM_I0_CLEAN_LED,0);
    }
    break;
  case DOCK_LED:
    {
      gpio_set_value(AM_I0_DOCK_LED,0);
    }
    break;
  case SPOT_LED:
    {
      gpio_set_value(AM_I0_SPOT_LED,0);
    }
    break;
  case ALL_LED:
    {
      gpio_set_value(AM_I0_CLEAN_LED,0);
      gpio_set_value(AM_I0_DOCK_LED,0);
      gpio_set_value(AM_I0_SPOT_LED,0);
    }
    break;
    
  }
}

/****************************************************************
*Function   :  display_power_up
*Description:  开机时的显示  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void display_power_up(void)
{
  static U8 time_cnt = 0;
  static U8 display_step = 0;
  
  if(time_cnt > 0)
  {
    time_cnt--;
    if(time_cnt == 0)
    {
      //AM_DEBUG_INFO("display_step=%d\r\n",display_step);
      if(++display_step >= 7)
      {
        display_power_up_flag = 0;
      }
    }
  }
  switch(display_step)
  {
  case 0:
    reset_led(ALL_LED);  
    set_led(DOCK_LED); 
    if(time_cnt == 0)
      time_cnt = FAST_TIME;
    break;
  case 1:
    reset_led(ALL_LED);  
    //set_led(SPOT_LED); 
    set_led(CLEAN_LED); 
    if(time_cnt == 0)
      time_cnt = SLOW_TIME;
    break;
  case 2:
    reset_led(ALL_LED);  
    //set_led(CLEAN_LED); 
    set_led(DOCK_LED); 
    if(time_cnt == 0)
      time_cnt = SLOW_TIME;
    break;
  case 3:
    reset_led(ALL_LED);  
    //set_led(DOCK_LED); 
    set_led(CLEAN_LED); 
    if(time_cnt == 0)
      time_cnt = SLOW_TIME;
    break; 
  case 4:
    reset_led(ALL_LED);  
    set_led(DOCK_LED); 
    set_led(SPOT_LED); 
    if(time_cnt == 0)
      time_cnt = SLOW_TIME;
    break; 
  case 5:
    reset_led(ALL_LED);  
    set_led(CLEAN_LED); 
    set_led(SPOT_LED); 
    if(time_cnt == 0)
      time_cnt = SLOW_TIME+20;
    break; 
  case 6:
    reset_led(ALL_LED);  
    set_led(ALL_LED);  
    if(time_cnt == 0)
      time_cnt = SLOW_TIME;
    break;
    
  }
}

/****************************************************************
*Function   :  display_testing
*Description:  UI状态为UI_TEST时的显示  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void display_testing(void)
{
	if (get_test_result() != 0)
  {
  	set_led(GREEN_LED);
  }
  else
  {
  	set_led(RED_LED);
  }
}

/****************************************************************
*Function   :  display_routine
*Description:  显示处理函数，根据UI和系统的不同状态作相应的处理
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void display_routine(void)
{  
  static int led_flash_cnt = 0;	 
  
  UI_STATE_E s = get_ui_state();
  U8 sleep_state = get_enter_sleep_state();
  sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
  if((sys_state_info_p.robot_state != ROBOT_STATE_WAITING)&&(display_power_up_flag == 1))
  {
    display_power_up_flag = 0;
  }
  
  if(display_power_up_flag == 1)
  {
    display_power_up();
    return;
  }
  
  led_flash_cnt++;
      
  reset_led(ALL_LED); 
  
  if (s == UI_TEST)
  {
  	display_testing();
    return;
  }
  else if(s == UI_ENTER_SLEEPING)
  {
    return;
  }
  else if (s == UI_ERROR)
  {
    set_led(RED_LED);
    return;
  }  
  
  switch(sys_state_info_p.robot_state)
  {
    
  case ROBOT_STATE_WAITING:
    { 
      set_led(ALL_LED);
    }
    break;
  case ROBOT_STATE_REMOTE_DRIVE:
  case ROBOT_STATE_CLEANING_ROOM:
    {
      if(sys_state_info_p.flush_state)
      {
        if(last_flush_state == 0)
          led_flash_cnt = HALF_LED_FLASH_TIME + 1;
        if(led_flash_cnt>HALF_LED_FLASH_TIME)
        {
          set_led(ALL_LED); 
        }
      }
      else if(sys_state_info_p.dock_state == 1)
      {
        set_led(ORANGE_LED); 
      }
      else
      {
        set_led(GREEN_LED);
      }
    }
    break;
    
  case ROBOT_STATE_SPOTTING:
  case ROBOT_STATE_WALLFOLLOW:
    {
      set_led(GREEN_LED);
    }
    break;
    
  case ROBOT_STATE_TESTING:
    {
      set_led(ALL_LED);
    }
    break;  
  case ROBOT_STATE_CHARGING:
    {
      if(sys_state_info_p.charging_state == CHARGING_ON)
      {
        if(led_flash_cnt>HALF_LED_FLASH_TIME)
        {
          set_led(ORANGE_LED);           
        }
      }
      else
      {
        set_led(GREEN_LED);
      }
    }
    break;         
  case ROBOT_STATE_PAUSE:
    if(led_flash_cnt>HALF_LED_FLASH_TIME)
    {
      set_led(ORANGE_LED);           
    }
    break;
  default:
    set_led(ALL_LED);  
    break;
    
  }
  
  if(led_flash_cnt > LED_FLASH_TIME)
    led_flash_cnt = 0;
  
  last_flush_state = sys_state_info_p.flush_state;
  
}

/****************************************************************
*Function   :  set_display_power_up_flag
*Description:  设置开机显示状态标志
*Input      :  是否开机状态
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void set_display_power_up_flag(U8 state)
{
  display_power_up_flag = state;
}

