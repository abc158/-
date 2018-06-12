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
#include "local_key_check.h"
#include "syscall_api.h"
#include "remote.h"
#include "act.h"
#include "display.h"
#include "am_key.h"
#include "sensor/sensor.h"
#include "ui-manager/exception.h"
#include "motor/robot_suction.h"
#include "ui-song-player.h"
#include "charge/charge.h"
#if defined(USE_WIFI_DEMO_1)
#include "wifi/wifi_demo1/simwifi.h"
#include "wifi/wifi_demo1/simsweep.h"
#elif defined(USE_WIFI_DEMO_2)
#include "wifi/wifi_demo2/simwifi_demo2.h"
#include "wifi/wifi_demo2/SimSweep_demo2.h"
#endif
#include "motor/robot_brush.h"
#define GO_SLEEP_TIME 500
#define WAITING_SLEEP_TIME 6000
#define FULLGO_START_TIME 200
#define WIFI_SLEEP_TIME    30000
typedef enum
{
  SPOT_MODE_NONE,
  SPOT_MODE_SPIRAL,/*螺旋清扫*/
  SPOT_MODE_GRID   /*局部弓字形*/
}SPOT_MODE_E;

static tm tmp_day;
static schedule_time_info tmp_schedule_info;
static UI_STATE_E ui_state;                   /*UI状态*/
static sys_state_info sys_state_info_p;       /*系统状态*/
static key_state_t *key_state_p;              /*物理按键状态指针*/
static ir_state_t *key_ir_p;                  /*物理按键状态指针*/

#if defined(USE_UART_WIFI)
static cmd_state_t *wifi_uart_cmd_state_p;   /*物理按键状态指针*/
#endif
static U8 enter_sleep_state = 0;              /*睡眠模式,有些方案可能要求不同睡眠模式*/
//static U8 fullgo_flag = 0;       
static U8 start_docking_flag = 0;             /*是否需要回座的标志*/
u8 ui_test_index = 0;                         /*测试项序号*/
U8 clean_index = 0;
TEST_CMD_E ui_test_cmd = CMD_TEST_WAIT;       /*测试项命令,根据ui_test_index从test_item_table[]中获得*/
u8 allow_self_test = 1;                       /*是否可以进入自检模式*/
static u16 goto_sleep_cnt = 0;                /*进入睡眠的时间计数*/
static u16 waiting_state_cnt = 0;
static u8 enhance_mode = 0;                   /*加强模式*/
static SPOT_MODE_E spot_mode = SPOT_MODE_NONE;/*局部清扫模式*/
static u8 board_key;                          /*物理按键*/
static u8 ir_key;                             /*遥控按键*/
static u16 fullgo_cnt;                        /*fullgo计时*/
#if defined(USE_WIFI_DEMO_2)
static U8 remote_state;
#endif
u8 direction_key_up_flag;//方向键松开标志位
u8 state_save =0;

static U8 adjust_side_brush_flag = 0;
static U8 wifi_flag = 0;
extern TEST_CMD_E test_item_table[];

void handle_act(U32 key_val);
extern cmd_state_t *get_ui_uart_cmd_state(void);
extern void sys_shutdown(void);
extern cmd_state_t *get_wifi_uart_cmd_state(void);
extern u8 battery_switch_check(void);
void direction_key_recover_handle(void);
extern void start_watchdog();
#if defined(USE_UART_WIFI)
	static U8 wifi_press_long_flag = 0;
#endif
UI_STATE_E get_ui_state(void)
{
  return ui_state;
}

void set_ui_state(UI_STATE_E s)
{
  ui_state = s;
}

U8 get_enter_sleep_state(void)
{
  return enter_sleep_state;
}

void set_enter_sleep_state(U8 state)
{
  enter_sleep_state = state;
}

void set_start_docking_state(u8 state)
{
  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))//在拖地模式下不能回座
  {
  	start_docking_flag = 0;
  }
  else
  {
	start_docking_flag = state;
  }	 
}

u8 get_start_docking_state(void)
{
  return start_docking_flag;
}
/****************************************************************
*Function   :  _act_init
*Description:  命令执行模块的初始化  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _act_init(void)
{
  key_state_p = get_key_state();
  key_ir_p    = get_ir_state();
  
  #if defined(USE_UART_WIFI)
  wifi_uart_cmd_state_p = get_wifi_uart_cmd_state();
  #endif
  #if defined(USE_WIFI_DEMO_1) || defined(USE_WIFI_DEMO_2)
  clear_map_index();
  #endif

}

/****************************************************************
*Function   :  _act_exit
*Description:  行为的退出  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _act_exit(void)
{
  ;
}

/****************************************************************
*Function   :  sleep_handle
*Description:  睡眠状态的处理  
*Input      :  按键消息
*Output     :  1：函数执行时处理了按键消息了；0：函数执行时没有处理按键消息
*Return     :  无
*Others     :  
******************************************************************/
u8 sleep_handle(U32 key)
{
	if (get_ui_state() == UI_ENTER_SLEEPING)
	{
		if (((key == (KEY_CLEAN | KEY_SHORT_UP))&&(board_key))||
            (key == (KEY_POWER | KEY_SHORT_UP))||
            (sys_state_info_p.robot_state == ROBOT_STATE_CHARGING)||
            (sys_state_info_p.robot_state == ROBOT_STATE_CLEANING_ROOM))
		{
           direction_key_up_flag =0; //清零该标志位，避免唤醒之后直接回到按方向键之前的状态	 
           robot_universl_wheel_init();
		   set_ui_state(UI_IDLE);
                   start_watchdog();
		  
		}
		else
		{
		  goto_sleep_cnt++;
		  if ((goto_sleep_cnt >= GO_SLEEP_TIME)&&
            ((sys_state_info_p.robot_state == ROBOT_STATE_WAITING)||
            (sys_state_info_p.robot_state == ROBOT_STATE_PAUSE)))
		  {
		  	goto_sleep_cnt = 0;
            printf("go to sleep####\r\n");
            robot_universl_wheel_exit();
		    act_command_q(CMD_POWER, CMD_RUN, NULL, 0);
		  }
		}
        return 1;
	}
	return 0;
}

/****************************************************************
*Function   :  ext_act_handle
*Description:  方案规格相关的行为处理  
*Input      :  按键消息
*Output     :  1：函数执行时处理了按键消息了；0：函数执行时没有处理按键消息
*Return     :  无
*Others     :  
******************************************************************/
u8 ext_act_handle(U32 key)
{
	UI_STATE_E s;
    U16 tmp_sleep_time = 0;
	s = get_ui_state();
	//是否打开墙检和地检
	if ( (sys_state_info_p.robot_state != ROBOT_STATE_WAITING &&
  	    sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)
  	    || (get_ui_state() == UI_TEST) )
  {
    robot_sensor_gather_start(1);
  }
  else
  {
    //robot_sensor_gather_start(1);
    robot_sensor_gather_start(0);
  }  
  //停止状态一段时间后进入睡眠
  if ((sys_state_info_p.robot_state == ROBOT_STATE_WAITING)||
    (sys_state_info_p.robot_state == ROBOT_STATE_PAUSE))
  {    
    if ((s != UI_ERROR) && (s != UI_TEST) && (s != UI_ENTER_SLEEPING))
    {
    	waiting_state_cnt++;
      if(wifi_flag > 0)
         tmp_sleep_time = WIFI_SLEEP_TIME;
      else
         tmp_sleep_time = WAITING_SLEEP_TIME;
      
      if (waiting_state_cnt >= tmp_sleep_time)
      {
         waiting_state_cnt = 0;
         if(wifi_flag > 0)
            wifi_flag = 0;
         set_ui_state(UI_ENTER_SLEEPING);
      }
    }
    else
    {
      waiting_state_cnt = 0;
    }        
  }
  else
  {
     waiting_state_cnt = 0;
  }
  //dock处理
    if (get_start_docking_state() == 1 && sys_state_info_p.robot_state != ROBOT_STATE_DOCK)
	{
		act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
		songplayer_play_id(SONG_ID_DOCKING_START, 0);
	  set_start_docking_state(0);
	}
	if((sys_state_info_p.robot_state == ROBOT_STATE_DOCK) && \
	     !gpio_get_value(AM_I0_WATER_TANK_DETECT))//拖地模式下不响应回座
	{
		act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0); 	
	}
	
    if(sys_state_info_p.robot_state == ROBOT_STATE_DOCK)
    {
        if(adjust_side_brush_flag == 0)
        {
            robot_side_brush_adjust_set(SIDE_BRUSH_MAX_VOLTAGE/3);
            adjust_side_brush_flag = 1;
        }
    }
    else 
    {
        if(adjust_side_brush_flag == 1)
        {
            adjust_side_brush_flag = 0;
			robot_side_brush_adjust_set(SIDE_BRUSH_MAX_VOLTAGE);
        }
    }
    if(sys_state_info_p.robot_state == ROBOT_STATE_CHARGING)
    {
        if(get_ui_error_num() != UI_ERROR_00_REVERSE)
        {
            clear_ui_error_number();      
            set_ui_state(UI_IDLE);
        }
    }
	//fullgo处理
	if (get_ui_state() == UI_FULLGO)
	{
	  if ( (sys_state_info_p.robot_state == ROBOT_STATE_WAITING) ||
	  	   ((sys_state_info_p.robot_state == ROBOT_STATE_CHARGING) && 
	  	    (sys_state_info_p.charging_state == CHARGING_COMPLETE)) ) 
	  {
	    fullgo_cnt++;
	    if (fullgo_cnt > FULLGO_START_TIME)
	    {
	      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
	      fullgo_cnt = 0;
	    }
	  }
	  else
	  {
	    fullgo_cnt = 0;
	  }
	}
	//sleep处理
	if (sleep_handle(key) == 1)
	{
	  return 1;
	}
	//故障处理
	exception_handle();
	
	return 0;
}

/****************************************************************
*Function   :  act_routine
*Description:  行为处理函数  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void act_routine(void)
{
  U8 i;
  U32 key_value = KEY_NULL;      
  
  //物理按键消息获取
  for(i=0; i<TOTAL_KEY_NUM; i++)
  {
    if(key_state_p[i].state != KEY_NULL)
    {
      key_value = key_state_p[i].state | key_state_p[i].keyval;
      key_state_p[i].state = KEY_NULL;//clear the message
      board_key = 1;
    } 
  }
  //遥控按键消息或者
  for(i=0; i<TOTAL_IR_NUM; i++)
  {
    if(key_ir_p[i].state != KEY_NULL)
    {
      key_value = key_ir_p[i].state | key_ir_p[i].keyval;
      key_ir_p[i].state = KEY_NULL;//clear the message 
      ir_key = 1;
    } 
  }
 
 
  #if defined(USE_UART_WIFI)
  for(i=0; i<UART_CMD_MAX; i++)
  {
     if(wifi_uart_cmd_state_p[i].state != KEY_NULL)
     {
       key_value = wifi_uart_cmd_state_p[i].state | wifi_uart_cmd_state_p[i].keyval;
       wifi_uart_cmd_state_p[i].state = KEY_NULL;//clear the message
     } 
  }
  #endif
 //规格相关的一些处理
	if (ext_act_handle(key_value) == 1)
	{
	  return;
	}
	//按键消息处理
  if ((key_value != KEY_NULL)&&(charging_jack() == 0))
  {
    handle_act(key_value);
    board_key = 0;
    ir_key = 0;
  }
  else
  {
     board_key = 0;
     ir_key = 0;
  }
  direction_key_recover_handle();
}

/****************************************************************
*Function   :  handle_act_waiting
*Description:  系统状态为ROBOT_STATE_WAITING的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_waiting(U32 key_val)
{  
  //printf("handle_act_waiting\r\n"); 
  
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      #if defined(USE_WIFI_DEMO_1) || defined(USE_WIFI_DEMO_2)
        clear_map_index();
      #endif

      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
      if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
      }
      else
      {
          songplayer_play_id(SONG_ID_START_MOP, 0);
      }
      break;
      
    case KEY_DOCK | KEY_SHORT_UP:
	  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	  {
	  	  break;
	  }
      act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_DOCKING_START, 0);
      break;
    
    case KEY_SPOT | KEY_SHORT_UP:
	  state_save_and_recover_grid(0);
      act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
      spot_mode = SPOT_MODE_SPIRAL;
      break;
    
    case KEY_GRID | KEY_SHORT_UP:
	  state_save_and_recover_grid(0);
      act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
      spot_mode = SPOT_MODE_GRID;
      break;
          
    case KEY_WALLFLOW | KEY_SHORT_UP:
      act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
      break;          
      
    case KEY_POWER | KEY_SHORT_UP:   
    	goto_sleep_cnt = GO_SLEEP_TIME;  
    	songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
    	sleep(10); 
      set_ui_state(UI_ENTER_SLEEPING);
      break;                                          
        
    default:
    	//printf("default=%x \r\n", s->cmd);
      break;         
  }    
}

/****************************************************************
*Function   :  handle_act_running
*Description:  系统状态为ROBOT_STATE_CLEANING_ROOM的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_running(U32 key_val)
{  
  //printf("handle_act_running\r\n"); 
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);   
      sleep(10);
	  songplayer_play_id(SONG_ID_ENTER_PAUSED, 0);
      break;
      
    case KEY_DOCK | KEY_SHORT_UP:
	  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	  {
	  	  break;
	  }	  
      act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_DOCKING_START, 0);
      break;
    
    case KEY_SPOT | KEY_SHORT_UP:
      state_save_and_recover_grid(0);
      act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
      spot_mode = SPOT_MODE_SPIRAL;
      break;
    
    case KEY_GRID | KEY_SHORT_UP:
      state_save_and_recover_grid(0);
      act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
      spot_mode = SPOT_MODE_GRID;
      break;
          
    case KEY_WALLFLOW | KEY_SHORT_UP:
      act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
      break;          
      
    case KEY_POWER | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_STOP, NULL, 0);
      songplayer_play_id(SONG_ID_NORMAL_CLEANING_STOP, 0);
      break;                                          
        
    default:
    	//printf("default=%x \r\n", s->cmd);
      break;         
  } 
      
}

/****************************************************************
*Function   :  handle_act_pausing
*Description:  系统状态为ROBOT_STATE_PAUSE的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_pausing(U32 key_val)
{  
  //printf("handle_act_pausing\r\n"); 
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
      if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
      }
      else
      {
          songplayer_play_id(SONG_ID_START_MOP, 0);
      }	  
      break;
      
    case KEY_DOCK | KEY_SHORT_UP:
	  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	  {
	  	  break;
	  }	  
      act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_DOCKING_START, 0);
      break;
    
    case KEY_SPOT | KEY_SHORT_UP:
	  state_save_and_recover_grid(0);
      act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
      spot_mode = SPOT_MODE_SPIRAL;
      break;
    
    case KEY_GRID | KEY_SHORT_UP:
	  state_save_and_recover_grid(0);
      act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
      spot_mode = SPOT_MODE_GRID;
      break;
          
    case KEY_WALLFLOW | KEY_SHORT_UP:
      act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
      break;          
    
    case KEY_POWER | KEY_SHORT_UP:
      goto_sleep_cnt = GO_SLEEP_TIME;  
      songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
      sleep(10); 
      set_ui_state(UI_ENTER_SLEEPING);
      break;                                          
        
    default:
    	//printf("default=%x \r\n", s->cmd);
      break;         
  }     
}

/****************************************************************
*Function   :  handle_act_spotting
*Description:  系统状态为ROBOT_STATE_SPOTTING的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_spotting(U32 key_val)
{  
  //printf("handle_act_spotting\r\n"); 
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
      if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
      }
      else
      {
          songplayer_play_id(SONG_ID_START_MOP, 0);
      }
      spot_mode = SPOT_MODE_NONE;
      break;
      
    case KEY_DOCK | KEY_SHORT_UP:
	  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	  {
	  	  break;
	  }	  
      act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_DOCKING_START, 0);
      spot_mode = SPOT_MODE_NONE;      
      break;
    
    case KEY_SPOT | KEY_SHORT_UP:
      state_save_and_recover_grid(0);
      if (spot_mode == SPOT_MODE_GRID)
      {
        act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
        songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
        spot_mode = SPOT_MODE_SPIRAL;
      }
      else
      {
		act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
      	songplayer_play_id(SONG_ID_ENTER_PAUSED, 0);
      	spot_mode = SPOT_MODE_NONE;
      }
      break;
    
    case KEY_GRID | KEY_SHORT_UP:
	  state_save_and_recover_grid(0);
      if (spot_mode == SPOT_MODE_SPIRAL)
      {
        act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
        songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
        spot_mode = SPOT_MODE_GRID;
      }
      else
      {
		act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
      	songplayer_play_id(SONG_ID_ENTER_PAUSED, 0);
      	spot_mode = SPOT_MODE_NONE;
      }
      break;
          
    case KEY_WALLFLOW | KEY_SHORT_UP:
      act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
      spot_mode = SPOT_MODE_NONE;
      break;          
    
    case KEY_POWER | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_STOP, NULL, 0);
      songplayer_play_id(SONG_ID_NORMAL_CLEANING_STOP, 0);
      spot_mode = SPOT_MODE_NONE;
      break;                                          
        
    default:
    	//printf("default=%x \r\n", s->cmd);
      break;         
  }    
}

/****************************************************************
*Function   :  handle_act_wall_following
*Description:  系统状态为ROBOT_STATE_WALLFOLLOW的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_wall_following(U32 key_val)
{  
  //printf("handle_act_wall_following\r\n"); 
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
      if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
      }
      else
      {
          songplayer_play_id(SONG_ID_START_MOP, 0);
      }
      break;
      
    case KEY_DOCK | KEY_SHORT_UP:
	  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	  {
	  	  break;
	  }	  
      act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_DOCKING_START, 0);
      break;
    
    case KEY_SPOT | KEY_SHORT_UP:
      state_save_and_recover_grid(0);
      act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
      spot_mode = SPOT_MODE_SPIRAL;
      break;
    
    case KEY_GRID | KEY_SHORT_UP:
      state_save_and_recover_grid(0);
      act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
      spot_mode = SPOT_MODE_GRID;
      break;
          
    case KEY_WALLFLOW | KEY_SHORT_UP:                  
      act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
      songplayer_play_id(SONG_ID_ENTER_PAUSED, 0);
      break;
    case KEY_POWER | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_STOP, NULL, 0);
      songplayer_play_id(SONG_ID_NORMAL_CLEANING_STOP, 0);
      break;                                          
        
    default:
    	//printf("default=%x \r\n", s->cmd);
      break;         
  }       
}

/****************************************************************
*Function   :  handle_act_remote_driving
*Description:  系统状态为ROBOT_STATE_REMOTE_DRIVE的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_remote_driving(U32 key_val)
{  
  //printf("handle_act_remote_driving\r\n");
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
	  clean_out_key_flg();
      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
      if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
      }
      else
      {
          songplayer_play_id(SONG_ID_START_MOP, 0);
      }	  
      break;
      
    case KEY_DOCK | KEY_SHORT_UP:
	  clean_out_key_flg();
	  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
	  {
	  	  break;
	  }	  
      act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_DOCKING_START, 0);
      break;
    
    case KEY_SPOT | KEY_SHORT_UP:
	  clean_out_key_flg();
	  state_save_and_recover_grid(0);
      act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
      spot_mode = SPOT_MODE_SPIRAL;
      break;
    
    case KEY_GRID | KEY_SHORT_UP:
	  clean_out_key_flg();
	  state_save_and_recover_grid(0);
      act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
      spot_mode = SPOT_MODE_GRID;
      break;
          
    case KEY_WALLFLOW | KEY_SHORT_UP:
	  clean_out_key_flg();
      act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
      break;          
      
    case KEY_POWER | KEY_SHORT_UP: 
	  clean_out_key_flg();
      act_command_q(CMD_CLEAN, CMD_STOP, NULL, 0);
      sleep(10);
      goto_sleep_cnt = GO_SLEEP_TIME;  
      songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
      sleep(10); 
      set_ui_state(UI_ENTER_SLEEPING);
      break; 
      break;                                          
    case KEY_DIRT | KEY_SHORT_UP:
	  clean_out_key_flg();
	  break;
    default:
      break;         
  }    
}

/****************************************************************
*Function   :  handle_act_docking
*Description:  系统状态为ROBOT_STATE_DOCK的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_docking(U32 key_val)
{  
  //printf("handle_act_remote_driving\r\n"); 
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
      if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
      }
      else
      {
          songplayer_play_id(SONG_ID_START_MOP, 0);
      }
      break;          
    
    case KEY_SPOT | KEY_SHORT_UP:
      state_save_and_recover_grid(0);
      act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
      spot_mode = SPOT_MODE_SPIRAL;
      break;
    
    case KEY_GRID | KEY_SHORT_UP:
	  state_save_and_recover_grid(0);  
      act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
      spot_mode = SPOT_MODE_GRID;
      break;
          
    case KEY_WALLFLOW | KEY_SHORT_UP:
      act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
      songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
      break;        
    
    case KEY_DOCK | KEY_SHORT_UP:
    case KEY_POWER | KEY_SHORT_UP:
      act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
      songplayer_play_id(SONG_ID_DOCKING_STOP, 0);
      break;                                          
        
    default:
    	//printf("default=%x \r\n", s->cmd);
      break;         
  }     
}

/****************************************************************
*Function   :  handle_act_charging
*Description:  系统状态为ROBOT_STATE_CHARGING的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_charging(U32 key_val)
{   
	//printf("handle_act_charging\r\n");
    if(battery_switch_check() == 0)
  {
	  printf("battery_check\r\n");
      return;
  }
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
      if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
      }
      else
      {
          songplayer_play_id(SONG_ID_START_MOP, 0);
      }
      break; 
	case KEY_WIFI | KEY_LONG:
       #if defined(USE_UART_WIFI) 
       if(wifi_press_long_flag == 0)
       {
           printf("USE_UART_WIFI!!\r\n");
           #if defined(USE_WIFI_DEMO_1)
           InsertExtCmd(RestoreFactorySet); //进入wifi配网模式
           set_reset_wifi_flag(1);
           #elif defined(USE_WIFI_DEMO_2)
            send_config_network_cmd(); //进入wifi配网模式
           #endif
           songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
           wifi_press_long_flag = 1;
           wifi_flag = 1;
       }
       #endif
       break; 
    case KEY_WIFI | KEY_LONG_UP:
    case KEY_WIFI | KEY_HOLD_UP:
      #if defined(USE_WIFI_DEMO_1) || defined(USE_WIFI_DEMO_2)
       wifi_press_long_flag = 0;
      #endif 
       break;	  
     case KEY_SPOT | KEY_SHORT_UP:
	 case KEY_DOCK | KEY_SHORT_UP:
	 case KEY_POWER | KEY_SHORT_UP:
	 case KEY_LEFT | KEY_SHORT_UP:
	 case KEY_RIGHT | KEY_SHORT_UP:
	 case KEY_FORWARD | KEY_SHORT_UP:
	 case KEY_DIRT | KEY_SHORT_UP:
	 case KEY_STOP | KEY_SHORT_UP:
	 case KEY_ADJUST | KEY_SHORT_UP:
	 case KEY_WALLFLOW | KEY_SHORT_UP: 
	 case KEY_GRID | KEY_SHORT_UP:
	 case KEY_SOS | KEY_SHORT_UP: 		      
        songplayer_play_id(SONG_ID_BUTTON_REJECT, 0);
      break;   
	default:
		break;
  }    
}

/****************************************************************
*Function   :  handle_act_testing
*Description:  UI状态为UI_TEST的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_testing(U32 key_val)
{   
	//printf("handle_act_testing\r\n"); 
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      ui_test_index++;
      if (ui_test_index >= UI_TEST_ITEM_MAX)
      {
        ui_test_index = 0;
      }
      ui_test_cmd = test_item_table[ui_test_index];
      break;                                       
        
    default:
      break;         
  }    
}

/****************************************************************
*Function   :  handle_act_ui_error
*Description:  UI状态为UI_ERROR的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_ui_error(U32 key_val)
{   
	//printf("handle_act_ui_error\r\n"); 
  switch(key_val)
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
      clear_ui_error_number();                                       
      set_ui_state(UI_IDLE);
      break;
        
    default:
      break;         
  }    
}

/****************************************************************
*Function   :  handle_act_fullgo
*Description:  UI状态为UI_FULLGO的按键响应函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_fullgo(U32 key_val)
{   
	//printf("handle_act_ui_fullgo\r\n");  
	if ((key_val == (KEY_CLEAN | KEY_SHORT_UP)) && board_key)
	{
	  act_command_q(CMD_CLEAN, CMD_STOP, NULL, 0);
    songplayer_play_id(SONG_ID_NORMAL_CLEANING_STOP, 0);
    set_ui_state(UI_IDLE);
    return;
	}
	
  if (key_val & KEY_SHORT_UP)
  {
    songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);   
  }
}

/****************************************************************
*Function   :  handle_act_common
*Description:  公共的按键响应函数，如果在各种状态下响应都一致的行为，可以放在这里来实现  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act_common(U32 key_val)
{
	switch(key_val)
	{
	  case KEY_LEFT | KEY_DOWN:
        #if defined(USE_WIFI_DEMO_2)
        remote_state = REMOTE_LEFT;
        #endif
    	send_left_cmd(CMD_RUN);
      break;
    
    case KEY_RIGHT | KEY_DOWN:
      #if defined(USE_WIFI_DEMO_2)
      remote_state = REMOTE_RIGHT;  
      #endif
      send_right_cmd(CMD_RUN);    
      break;
    
    case KEY_FORWARD | KEY_DOWN:
      
      send_forward_cmd(CMD_RUN);
      #if defined(USE_WIFI_DEMO_2)
      remote_state = REMOTE_FORWARD;
      #endif
      break;
      
    case KEY_BACKWARD | KEY_DOWN:
     
      send_backward_cmd(CMD_RUN);
      #if defined(USE_WIFI_DEMO_2)
      remote_state = REMOTE_BACK;
      #endif
      break;
    
    case KEY_LEFT | KEY_SHORT_UP:
    case KEY_LEFT | KEY_LONG_UP:
    case KEY_LEFT | KEY_HOLD_UP:
      send_left_cmd(CMD_STOP);
      #if defined(USE_WIFI_DEMO_2)
      remote_state = REMOTE_IDLE;
      #endif
      break;
    
    case KEY_RIGHT | KEY_SHORT_UP:
    case KEY_RIGHT | KEY_LONG_UP:
    case KEY_RIGHT | KEY_HOLD_UP:
      send_right_cmd(CMD_STOP);
      #if defined(USE_WIFI_DEMO_2)
      remote_state = REMOTE_IDLE;
      #endif
      break;
    
    case KEY_FORWARD | KEY_SHORT_UP:
    case KEY_FORWARD | KEY_LONG_UP:
    case KEY_FORWARD | KEY_HOLD_UP:
      send_forward_cmd(CMD_STOP);
      #if defined(USE_WIFI_DEMO_2)
      remote_state = REMOTE_IDLE;
      #endif
      break;
      
    case KEY_BACKWARD | KEY_SHORT_UP:
    case KEY_BACKWARD | KEY_LONG_UP:
    case KEY_BACKWARD | KEY_HOLD_UP:
      send_backward_cmd(CMD_STOP);
      #if defined(USE_WIFI_DEMO_2)
      remote_state = REMOTE_IDLE;
      #endif
      break;        
    
    case KEY_ADJUST | KEY_SHORT_UP:
	  if((sys_state_info_p.robot_state != ROBOT_STATE_DOCK) && \
		(sys_state_info_p.robot_state != ROBOT_STATE_WAITING) && \
		  (sys_state_info_p.robot_state != ROBOT_STATE_PAUSE) && \
			(sys_state_info_p.robot_state != ROBOT_STATE_REMOTE_DRIVE) && \
			gpio_get_value(AM_I0_WATER_TANK_DETECT))
      {
          if (enhance_mode)
          {
              enhance_mode = 0;          
              printf("normal ####\r\n");
              robot_suction_adjust_set(VACUUM_NORMAL_VOLTAGE);
              songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
          }
          else
          {
             enhance_mode = 1;      
             printf("fast #####\r\n");
             robot_suction_adjust_set(VACUUM_ENHANCE_VOLTAGE);
             songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
          }
      }
      break;
    
    case KEY_SOS | KEY_SHORT_UP:
      //send_backward_cmd(CMD_STOP);
      if (get_songplayer_onoff() == 1)
      {
        set_songplayer_onoff(0);
      }
      else
      {
        set_songplayer_onoff(1);
        songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
      }  
      break;
    case KEY_CLOCK | KEY_SHORT_UP:
      //设置时间前先赋值    
      tmp_day.week    = 1;//星期取值范围0-6,0代表星期天，6代表星期六
      tmp_day.hour    = 12;
      tmp_day.min     = 12;
      act_command_q((U32)CMD_CLOCK, (U8)CMD_RUN, (void *)&tmp_day, sizeof(tm));
     // set_current_time_info(&set_time);
      break;
    case KEY_SCHEDULE | KEY_SHORT_UP:
       //设置预约示例 
       tmp_schedule_info.SCH_ENABLE.BYTE = 0x1;//设置为星期天预约有效
       tmp_schedule_info.t[0].hour = 8;
       tmp_schedule_info.t[0].min  = 8;
       act_command_q(CMD_SCHDULE, CMD_RUN, (void *)&tmp_schedule_info, sizeof(schedule_time_info));
       //sleep(10);
       //sys_shutdown(); //关闭DC-DC
      break;
	case KEY_WIFI | KEY_LONG:
       #if defined(USE_UART_WIFI) 
       if(wifi_press_long_flag == 0)
       {
           printf("USE_UART_WIFI!!\r\n");
           #if defined(USE_WIFI_DEMO_1)
           InsertExtCmd(RestoreFactorySet); //进入wifi配网模式
           set_reset_wifi_flag(1);
           #elif defined(USE_WIFI_DEMO_2)
            send_config_network_cmd(); //进入wifi配网模式
           #endif
           songplayer_play_id(SONG_ID_BUTTON_CLICK, 0);
           wifi_press_long_flag = 1;
           wifi_flag = 1;
       }
       #endif
       break; 
    case KEY_WIFI | KEY_LONG_UP:
    case KEY_WIFI | KEY_HOLD_UP:
      #if defined(USE_WIFI_DEMO_1) || defined(USE_WIFI_DEMO_2)
       wifi_press_long_flag = 0;
      #endif 
       break;
    default:
    	break;
  }
}

/****************************************************************
*Function   :  handle_act
*Description:  按键消息的处理函数  
*Input      :  按键消息
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void handle_act(U32 key_val)
{	
	UI_STATE_E s;
	
	s = get_ui_state();

	if ((key_val == (KEY_CLEAN | KEY_HOLD)) && allow_self_test && board_key) 
	{
	  set_ui_state(UI_TEST);
	  allow_self_test = 0;
	  ui_test_cmd = test_item_table[ui_test_index];
	  printf("enter self test\r\n"); 
	}
	if ((key_val == (KEY_CLEAN | KEY_HOLD)) && allow_self_test && ir_key) 
	{
	  set_ui_state(UI_FULLGO);
	  allow_self_test = 0;
	  fullgo_cnt = FULLGO_START_TIME;
	  printf("enter fullgo\r\n"); 
	}
	
	if (s == UI_TEST)
	{	
	  handle_act_testing(key_val);
	  return;
	}	
	else if (s == UI_ERROR)
	{
	  handle_act_ui_error(key_val);
	  return;
	}
	else if (s == UI_FULLGO)
	{
	  handle_act_fullgo(key_val);
	  return;
	}
		
			
  switch (sys_state_info_p.robot_state)
  {
    case ROBOT_STATE_WAITING:
      handle_act_waiting(key_val);
    	break;
    
    case ROBOT_STATE_CLEANING_ROOM:
      handle_act_running(key_val);
    	break;
    
    case ROBOT_STATE_PAUSE:
      handle_act_pausing(key_val);
    	break;
    	
    case ROBOT_STATE_WALLFOLLOW:
      handle_act_wall_following(key_val);
    	break;
    
    case ROBOT_STATE_SPOTTING:
      handle_act_spotting(key_val);
    	break;
    		
    case ROBOT_STATE_REMOTE_DRIVE:
      handle_act_remote_driving(key_val);
    	break;
    
    case ROBOT_STATE_DOCK:
      handle_act_docking(key_val);
    	break;
    		
    case ROBOT_STATE_CHARGING:
      handle_act_charging(key_val);
    	break;    	    
    		
    default:
    	break;
  }
  
  switch(key_val)//避免在方向键松开之后等待恢复原来的状态的时候按其他键造成的bug,放到这里来是因为按局部键之间可能要记录方向键之前的状态
  { 
    case KEY_CLEAN | KEY_SHORT_UP:
    case KEY_DOCK | KEY_SHORT_UP:
    case KEY_SPOT | KEY_SHORT_UP:
    case KEY_GRID | KEY_SHORT_UP:
    case KEY_WALLFLOW | KEY_SHORT_UP:
    case KEY_POWER | KEY_SHORT_UP: 
	  direction_key_up_flag =0;
    default:
    	break;
  }
  
  if(sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)
  {
      handle_act_common(key_val);
  }
}

u8 last_state = 0;
extern void print_touch(void);
extern void print_cliff(void);
/****************************************************************
*Function   :  ui_handle_idle
*Description:  idle函数  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void ui_handle_idle(void)
{
  sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
  if (last_state != sys_state_info_p.robot_state)
	{
	    printf("new_state:%d \r\n", sys_state_info_p.robot_state);
	}
	//print_touch();
	last_state = sys_state_info_p.robot_state;
}

U8 get_remote_state(void)
{
    #if defined(USE_WIFI_DEMO_2)
    return remote_state;
    #else
    return 0;
    #endif
}
/****************************************************************
*Function   :  get_robot_current_state
*Description:  获取系统当前状态  
*Input      :  无
*Output     :  无
*Return     :  返回系统当前状态
*Others     :  by Ljh
******************************************************************/
u8 get_robot_current_state(void)
{
    sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
	return sys_state_info_p.robot_state;
	
}
/****************************************************************
*Function   :  direction_key_recover_handle
*Description:  对松开方向键的处理 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  by ljh
******************************************************************/
void direction_key_recover_handle(void)
{
	static uint32_t one_ms =0;
	if(direction_key_up_flag)
	{	  
	  	if(timer_ms()>(one_ms+700))//松开方向键700ms之后使状态跳回第一次按键前的状态
		{
			state_recover();
			direction_key_up_flag =0;
		}
		return;	  	  
	}
	one_ms = timer_ms();
	
}

/****************************************************************
*Function   :  cleaning_complete_handle
*Description:  对清扫完成的处理 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  by ljh
******************************************************************/
void cleaning_complete_handle(void)
{	
	if(sys_state_info_p.robot_state == ROBOT_STATE_SPOTTING)
	{
		sleep(10);
		state_save_and_recover_grid(1);
		return;
	}
	else
	{
		songplayer_play_id(SONG_ID_NORMAL_CLEANING_COMPLETE, 1);
	}
	
}

/****************************************************************
*Function   :  state_save_and_recover_grid
*Description:  状态保存与状态恢复(用于局部清扫) 
*Input      :  0:保存当前状态   1：恢复保存的状态  2:只返回保存的状态
*Output     :  无
*Return     :  无
*Others     :  by ljh
******************************************************************/
void state_save_and_recover_grid(u8 flag)
{
	static u8 state;
	if(flag ==0)
	{
		state = sys_state_info_p.robot_state;	
		printf("save_state_grid=%d\r\n",state);
	}
	if(flag ==1)
	{
	  printf("recover_state_grid=%d\r\n",state);
	  switch(state)
	  { 
		case ROBOT_STATE_CLEANING_ROOM:
		  act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
		  if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
		  {
			  songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
		  }
		  else
		  {
			  songplayer_play_id(SONG_ID_START_MOP, 0);
		  }	 		  
		  break;

		case ROBOT_STATE_WALLFOLLOW:
		  act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
		  songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);
		  break;  
		  
		case ROBOT_STATE_DOCK:
		  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
		  {
			  break;
		  }		  
		  act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
		  songplayer_play_id(SONG_ID_DOCKING_START, 0);
		  break;	
		default:
		  act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
		  sleep(10);
		  songplayer_play_id(SONG_ID_NORMAL_CLEANING_COMPLETE, 1);//清扫完成语音
		  break;         
	  }    			
	
	}
}

/****************************************************************
*Function   :  state_recover
*Description:  保存状态
*Input      :  无 
*Output     :  无
*Return     :  无
*Others     :  by ljh
******************************************************************/
void set_save_state(u8 s)
{
  state_save = s;
  printf("state_save:%d\r\n",state_save);
}

/****************************************************************
*Function   :  state_recover
*Description:  获取保存的状态
*Input      :  无 
*Output     :  无
*Return     :  保存好的状态
*Others     :  by ljh
******************************************************************/
u8 get_save_state(void)
{
  return state_save;
}

/****************************************************************
*Function   :  state_recover
*Description:  状态恢复
*Input      :  无 
*Output     :  无
*Return     :  无
*Others     :  by ljh
******************************************************************/
void state_recover(void)
{ 
  	u8 state;
	state = get_save_state();
    printf("recover_state=%d\r\n",state);
	switch(state)
	{		
	case ROBOT_STATE_WALLFOLLOW:		  
		  act_command_q(CMD_WALL_FOLLOW, CMD_RUN, NULL, 0);
		  songplayer_play_id(SONG_ID_WALLFOLLOW_CLEANING_START, 0);		  		  
	  break;		  
	  
	case ROBOT_STATE_SPOTTING:
		  if (spot_mode == SPOT_MODE_SPIRAL)
		  {
			act_command_q(CMD_SPOT, CMD_RUN, NULL, 0);
			songplayer_play_id(SONG_ID_KEYSTONE_CLEANING, 0);
		  }
		  else
		  {
			act_command_q(CMD_GRID1, CMD_RUN, NULL, 0);
			songplayer_play_id(SONG_ID_SPOT_CLEANING_START, 0);
		  }	  		  
	  break;
	case ROBOT_STATE_CLEANING_ROOM:
		  act_command_q(CMD_CLEAN, CMD_RUN, NULL, 0);
		  if(gpio_get_value(AM_I0_WATER_TANK_DETECT))
		  {
			  songplayer_play_id(SONG_ID_NORMAL_CLEANING_START, 0);
		  }
		  else
		  {
			  songplayer_play_id(SONG_ID_START_MOP, 0);
		  }	 	  		  
		  break;		  		  

	case ROBOT_STATE_PAUSE:
//		  robot_suction_ctrl(1);	//关风机
//		  robot_sidebrush_vols_set(0);//关边刷
//		  robot_midbrush_vols_set(0);//关主刷
//		  printf("robot stop\r\n");		  
	  break;	 		  
	case ROBOT_STATE_DOCK:
		  if(!gpio_get_value(AM_I0_WATER_TANK_DETECT))
		  {
			  break;
		  }	  
		  act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);
		  songplayer_play_id(SONG_ID_DOCKING_START, 0);		  		  
	  break;
	default:
	  break;		  
	}
	set_save_state(0);
}
