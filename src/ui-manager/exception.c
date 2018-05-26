//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "syscall_api.h"
#include "am_date_base.h"
#include "am_robot_type.h"
#include "am_config_macro.h"
#include "act.h"
#include "lib.h"
#include "ui-commands.h"
#include "ui-song-player.h"
#include "motor/robot_brush.h"

#define MAIN_BRUSH_CURRENT_HIGHT_TIMEOUT 600
#define SIDE_BRUSH_CURRENT_HIGHT_TIMEOUT 200
#define VAC_CURRENT_HIGHT_TIMEOUT        300 //3s
#define WHEEL_DROP_TIMEOUT               3000
#define BUMPER_BUMP_TIMEOUT              600  //6s
#define NOT_BUSTBIN_TIMEOUT             1000 //10s
#define STASIS_STALL_TIMEOUT            15000 //150s
#define CHECK_TIME                      4000 //40s

#define PLAY_ERROR_INTERVAL 1000 //10s

static uint16_t main_brush_current_over_count = 0;          /*主刷电流异常计数*/
static uint16_t side_brush_current_over_count = 0;          /*边刷电流异常计数*/
static uint16_t vac_current_over_count = 0;                 /*真空异常计数*/
static uint16_t vac_current_null_count = 0;                 /*真空异常计数*/
static uint16_t bumper_bump_count = 0;                      /*检测到碰撞发生的计数*/
static UI_ERROR_NUMBERS_E g_error_id = UI_ERROR_00_REVERSE; /*错误号*/
static sys_state_info sys_state_info_p;                     /*系统状态*/
static u16 play_err_cnt = 0;                                /*播放故障提示的时间间隔计数*/
static u16 play_err_num = 0;                                /*故障提示的次数*/
static u8 side_brush_stop = 0;                              /*边刷是否停住*/
static u32 sb_stop_start_time;                         /*边刷停止转动的开始时间*/
static u16 stop_act_cnt = 0;
extern u8 robot_is_maybe_not_bustbin(void);

void set_ui_error_number(UI_ERROR_NUMBERS_E id)
{
  g_error_id = id;
}

UI_ERROR_NUMBERS_E get_ui_error_num(void)
{
	return g_error_id;
}

void clear_ui_error_number(void)
{
  g_error_id = UI_ERROR_00_REVERSE;
}

u8 main_brush_current_is_stall(void)
{
  return (get_adc_chan_val(ADC_CHAN_MAIN_BRUSH_CURRENT) > MAIN_BRUSH_TWINE_CURRENT);
}

u8 side_brush_current_is_stall(void)
{
  return (get_adc_chan_val(ADC_CHAN_SIDE_BRUSH_CURRENT) > SIDE_BRUSH_TWINE_CURRENT);
}

u8 vaccum_is_maybe_stall(void)
{
  return (get_adc_chan_val(ADC_CHAN_VAC_CURENT) > 1000);
}

/****************************************************************
*Function   :  check_cliff_alwayon
*Description:  判断是否一直检测到悬崖  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
static void check_cliff_alwayon(void)
{
  static u32 cliff_front_right  = 0;
  static u32 cliff_front_left   = 0;
  static u32 cliff_side_right   = 0;
  static u32 cliff_side_left    = 0;
  static u32 cliff_error        = 0;
  int flag = 0;
  
  if(get_cliff_state() & CLIFF_FRONT_RIGHT)
  {
    cliff_front_right++;
    
    if(cliff_front_right > CHECK_TIME)
    {
      flag++;
    }
  }
  else
  {
    cliff_front_right = 0;
  }
  
  if(get_cliff_state() & CLIFF_FRONT_LEFT)
  {
    cliff_front_left++;
    if(cliff_front_left > CHECK_TIME)
    {
      flag++;
    }
  }
  else
  {
    cliff_front_left = 0;
  }
  
  if(get_cliff_state() & CLIFF_SIDE_RIGHT)
  {
    cliff_side_right++;
    if(cliff_side_right > CHECK_TIME)
    {
      flag++;
    }
  }
  else
  {
    cliff_side_right=0;
  }
  
  if(get_cliff_state() & CLIFF_SIDE_LEFT)
  {
    cliff_side_left++;
    if(cliff_side_left > CHECK_TIME)
    {
      flag++;
    }
  }
  else
  {
    cliff_side_left=0;
  }
  
  if( (flag < 4) && (flag > 0) )
  {
    set_ui_error_number(UI_ERROR_12_CLIFF_ERROR);
    cliff_error=1;
  }
  else if(cliff_error)
  {
    clear_ui_error_number();
    cliff_error = 0;
  }
  else
  {
    cliff_error = 0;
  }
}

/****************************************************************
*Function   :  sys_exception_monitor
*Description:  机器故障的检测函数  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void sys_exception_monitor(void)
{
  static u32 delay = 0;
  static uint8_t  begin_to_check = 0;

  sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
  
  if(!IS_CLEANNING_MODE(sys_state_info_p.robot_state))
  {
    delay = 0;
    begin_to_check = 0;
    main_brush_current_over_count = 0;
    side_brush_current_over_count = 0;
    vac_current_null_count = 0;
    vac_current_over_count = 0;
    bumper_bump_count = 0;

    return;
  }
  else
  {
    delay++;
    if(delay >= 300) // after 3s
    {
      delay = 0;
      begin_to_check = 1;
    }
  }
  
  if(!begin_to_check)
  {
    return;
  }
  
  check_cliff_alwayon();
  
  //主刷被缠住
  if ( (main_brush_current_is_stall())) {
    main_brush_current_over_count += 1;
    if (main_brush_current_over_count > MAIN_BRUSH_CURRENT_HIGHT_TIMEOUT) {
      set_ui_error_number(UI_ERROR_02_MAIN_BRUSH_ENTWINED);
    }
  } else {
    main_brush_current_over_count = 0;
  }
  //边刷被缠住
  if (side_brush_current_is_stall()) {
    side_brush_current_over_count += 1;
    //printf("side_brush_current_over_count=%d\r\n",side_brush_current_over_count);
    if (side_brush_current_over_count > SIDE_BRUSH_CURRENT_HIGHT_TIMEOUT) {
      robot_sidebrush_vols_set(0);
      sb_stop_start_time = timer_ms();
      side_brush_stop = 1;
    }
  } else {
    side_brush_current_over_count = 0;
    if ( (side_brush_stop == 1) && ( timer_elapsed(sb_stop_start_time) > 5000) )
    {
      robot_sidebrush_vols_set(SIDE_BRUSH_MAX_VOLTAGE);
      side_brush_stop = 0;
    }
  }
  //真空被堵住
  if(vaccum_is_maybe_stall())
  {
    vac_current_over_count++;
    if (vac_current_over_count > VAC_CURRENT_HIGHT_TIMEOUT) {
      set_ui_error_number(UI_ERROR_15_VACCUM_MOTO_ENTWINED);
    }
  }else {
    vac_current_over_count = 0;
  }
  //没装入垃圾盒？
  if(robot_is_maybe_not_bustbin())
  {
    vac_current_null_count++;
    if (vac_current_null_count >= NOT_BUSTBIN_TIMEOUT) {
      set_ui_error_number(UI_ERROR_10_DUSTBIN_UN_INSERT);
    }
  }else {
    vac_current_null_count = 0;
  }
  //碰撞档板故障
  if(robot_bump_mask()!=BUMP_MASK_NONE)
  {
    bumper_bump_count++;
    if (bumper_bump_count > BUMPER_BUMP_TIMEOUT) {
      set_ui_error_number(UI_ERROR_09_BUMP_ERROR);
    }
  }else {
    bumper_bump_count=0;
  }  
}

/****************************************************************
*Function   :  exception_handle
*Description:  机器故障的处理函数  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void exception_handle(void)
{
  UI_ERROR_NUMBERS_E t_error;
  u8 voice_id_error_base = VOICE_ID_ERROR_00_REVERSE;  
  t_error = get_ui_error_num();
  
  if (t_error != UI_ERROR_00_REVERSE)
  { 
  	if (IS_CLEANNING_MODE(sys_state_info_p.robot_state))
  	{
  	  set_ui_state(UI_ERROR);
      if(stop_act_cnt == 0)
      {
  	      act_command_q(CMD_CLEAN, CMD_STOP, NULL, 0);
          stop_act_cnt = 50;
      }
  	  play_err_cnt = PLAY_ERROR_INTERVAL - 5;  	  
  	}  	
    if(stop_act_cnt > 0)
      stop_act_cnt--;
  	play_err_cnt++;
  	if (play_err_cnt >= PLAY_ERROR_INTERVAL)
  	{	
  		play_err_cnt = 0;
      songplayer_play_id(voice_id_error_base+t_error, 0);
      play_err_num++;
    }
  }
  else
  {
     stop_act_cnt = 0;
  }
}