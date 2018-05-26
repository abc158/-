//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "ui-config.h"
#include "local_key_check.h"
#include "remote.h"
#include "ui-commands.h"
#include "am_config_macro.h"
#include "act.h"
#include "syscall_api.h"

static U16 ir_press_on[TOTAL_IR_NUM];  /*遥控按键按下的次数*/
static U16 ir_press_off[TOTAL_IR_NUM]; /*遥控按键释放的次数*/

/*遥控按键定义*/
static ir_state_t ir_press_state[TOTAL_IR_NUM]=
{	
  {KEY_NULL, (U32) IR_CLEAN,       KEY_CLEAN},
  {KEY_NULL, (U32) IR_DOCK,        KEY_DOCK},
  {KEY_NULL, (U32) IR_SPOT,        KEY_SPOT},
  {KEY_NULL, (U32) IR_POWER,       KEY_POWER},
  {KEY_NULL, (U32) IR_LEFT,        KEY_LEFT},
  {KEY_NULL, (U32) IR_RIGHT,       KEY_RIGHT},
  {KEY_NULL, (U32) IR_FORWARD,     KEY_FORWARD},
  {KEY_NULL, (U32) IR_SCHEDULE,    KEY_WIFI},
  {KEY_NULL, (U32) IR_CLOCK,       KEY_CLOCK},
  {KEY_NULL, (U32) IR_DRIVERSTOP,  KEY_STOP}, 
  {KEY_NULL, (U32) IR_BACKWARD,    KEY_BACKWARD} ,
  {KEY_NULL, (U32) IR_ADJUST,        KEY_ADJUST} ,  
  {KEY_NULL, (U32) IR_WALLFOLLOW,  KEY_WALLFLOW} ,
  {KEY_NULL, (U32) IR_GRID,        KEY_GRID} , 
  {KEY_NULL, (U32) IR_SOUND,       KEY_SOS} ,     
};

extern void ir_rx_init(void);
extern uint8_t get_ir_local(void);

/****************************************************************
*Function   :  _remote_init
*Description:  遥控按键初始化  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _remote_init(void)
{
  U8 i;
  
  for(i = 0; i < TOTAL_IR_NUM; i++)
  {
    ir_press_state[i].state = KEY_NULL;
    ir_press_on[i]  = 0;
    ir_press_off[i] = 0;
  } 
  ir_rx_init();
}

/****************************************************************
*Function   :  _remote_exit
*Description:  遥控按键退出 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _remote_exit(void)
{
  ;
}

/****************************************************************
*Function   :  remote_routine
*Description:  遥控按键的解码和处理  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void remote_routine(void)
{
  ir_rx_decode();
  handle_remote();
}


/****************************************************************
*Function   :  handle_remote
*Description:  遥控按键检测，记录按键消息 
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
static void handle_remote( void )
{
  U16 ir_val = KEY_NULL;  
  U8 i = 0;
  
  ir_val = get_ir_local();
  for(i=0; i<TOTAL_IR_NUM; i++)
  {
    if (ir_val == ir_press_state[i].ir_code)
    {
      ir_press_on[i]++; 
      ir_press_off[i] = 0;    
      if (ir_press_on[i] >= 4 && ir_press_on[i] < 200)
      {
        ir_press_state[i].state = KEY_DOWN;
      }
      else if (ir_press_on[i] >= 200 && ir_press_on[i] < 400)
      {
        ir_press_state[i].state = KEY_LONG;
      }
      else if (ir_press_on[i] >= 400)
      {
        ir_press_state[i].state = KEY_HOLD;
        ir_press_on[i] = 400;
      }
    }
    else
    {
      ir_press_off[i]++;
      if (ir_press_off[i] >= 4)
      {
        if (ir_press_on[i] >= 4 && ir_press_on[i] < 200)
        {
          ir_press_state[i].state = KEY_SHORT_UP;          
        } 
        else if (ir_press_on[i] >= 200 && ir_press_on[i] < 400)
        {
          ir_press_state[i].state = KEY_LONG_UP; 
        }
        else if (ir_press_on[i] >= 400)
        {
          ir_press_state[i].state = KEY_HOLD_UP; 
        }
        ir_press_on[i] = 0;         
      }
    }
  }
} 

/****************************************************************
*Function   :  get_ir_state
*Description:  返回遥控按键存放的地址
*Input      :  无
*Output     :  遥控按键消息存放的地址
*Return     :  无
*Others     :  
******************************************************************/
ir_state_t *get_ir_state(void)
{
  return &ir_press_state[0];
}




