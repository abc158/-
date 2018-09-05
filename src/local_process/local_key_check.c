//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "local_key_check.h"
#include "am_key.h"
#include "ui-commands.h"
#include "am_date_base.h"
#include "act.h"
#include "syscall_api.h"

static U16 key_press_on[TOTAL_KEY_NUM];  /*按键按下的次数*/
static U8 key_press_off[TOTAL_KEY_NUM];  /*按键抬起的次数*/

/*物理按键定义*/
static key_state_t key_press_state[TOTAL_KEY_NUM]=
{	
  {PRESS_NULL, AM_IO_MC_KEY_WAKE_ON, 0, KEY_CLEAN}	,
  {PRESS_NULL, AM_IO_MC_KEY_DOCK, 0, KEY_DOCK },
  {PRESS_NULL, AM_IO_MC_KEY_WALLFOLLOW, 0, KEY_WALLFLOW },
  {PRESS_NULL, AM_IO_BUTTON_CLOCK, 0, KEY_NULL},
  {PRESS_NULL, AM_IO_BUTTON_CLEAN, 0, KEY_NULL}
};

/****************************************************************
*Function   :  key_single_init
*Description:  申请物理按键使用的IO  
*Input      :  无
*Output     :  无用
*Return     :  无
*Others     :  
******************************************************************/
U8 key_single_init()
{
  int ret = -1;
  for( int i=0; i<TOTAL_KEY_NUM; i++ )
  {
    ret = gpio_request_one(key_press_state[i].am_io, GPIO_F_DIR_IN);
    
    if(ret != -1)
    {
      ;
    }
    else
    {
      printf("key io not ok\r\n");
    }
  }  
  return 0;
}

/****************************************************************
*Function   :  key_single_init
*Description:  释放物理按键使用的IO  
*Input      :  无
*Output     :  无用
*Return     :  无
*Others     :  
******************************************************************/
U8 key_single_exit()
{
  for( int i=0; i<TOTAL_KEY_NUM; i++ )
  {
    gpio_free(key_press_state[i].am_io);
  }     
  return 0;
}

/****************************************************************
*Function   :  _key_init
*Description:  按键检测初始化  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _key_init(void)
{  
  U8 i;

  key_single_init();
  for(i=0; i<TOTAL_KEY_NUM; i++)
  {
    key_press_state[i].state = PRESS_NULL;
    key_press_on[i]          = 0;
    key_press_off[i]         = 0;
  }
}

/****************************************************************
*Function   :  _key_exit
*Description:  按键退出  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void _key_exit(void)
{
  key_single_exit();
}


/****************************************************************
*Function   :  key_routine
*Description:  按键检测，记录按键消息  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void key_routine(void)
{ 
	u8 i;
	u8 gpio_state;
	
  //read all gpio
  for(i=0; i<TOTAL_KEY_NUM; i++)
  {
    if(key_press_state[i].keyval==KEY_NULL)
    {
      continue;
    }
    else
    {
      gpio_state = gpio_get_value(key_press_state[i].am_io);
      if (gpio_state == key_press_state[i].input_gpio_state)
      { 
      printf("11111111111111111111\r\n");
        key_press_on[i]++; 
        key_press_off[i] = 0;    
        if (key_press_on[i] >= 4 && key_press_on[i] < 200)
        {
          key_press_state[i].state = KEY_DOWN;
        }
        else if (key_press_on[i] >= 200 && key_press_on[i] < 400)
        {
          key_press_state[i].state = KEY_LONG;
        }
        else if (key_press_on[i] >= 400)
        {
          key_press_state[i].state = KEY_HOLD;
          key_press_on[i] = 400;
        }
      } 
      else
      {
        key_press_off[i]++;
        if (key_press_off[i] >= 4)
        {
          if (key_press_on[i] >= 4 && key_press_on[i] < 200)
          {
            key_press_state[i].state = KEY_SHORT_UP;          
          } 
          else if (key_press_on[i] >= 200 && key_press_on[i] < 400)
          {
            key_press_state[i].state = KEY_LONG_UP; 
          }
          else if (key_press_on[i] >= 400)
          {
            key_press_state[i].state = KEY_HOLD_UP; 
          }
          key_press_on[i] = 0;         
        }
      }                     
    }
  }  
}

/****************************************************************
*Function   :  get_key_state
*Description:  返回物理按键存放的地址
*Input      :  无
*Output     :  物理按键消息存放的地址
*Return     :  无
*Others     :  
******************************************************************/
key_state_t *get_key_state(void)
{
  return &key_press_state[0];
}

/****************************************************************
*Function   :  any_key
*Description:  是否有物理按键按下 
*Input      :  无
*Output     :  1：有按键 0：无按键
*Return     :  无
*Others     :  
******************************************************************/
u8 any_key(void)
{
  u8 i = 0;
  
  for(i=0; i<TOTAL_KEY_NUM; i++)
  {
    if(key_press_state[i].state != PRESS_NULL)
    {
      //key_press_state[i].state = PRESS_NULL;
      return 1;
    }
  }
  return 0;
}
