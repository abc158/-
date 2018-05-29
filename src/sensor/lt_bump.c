#include "am_type.h"
#include "sensor/sensor.h"
#include "syscall_api.h"
#include "am_config_macro.h"

#define max(a,b) ((a)>(b)?(a):(b))

#define TOUCH_RIGHT_MOVE 2

#define TOUCH_BUMP_TRIGGER_TIME 120

#define LIGHT_BUMP_DELAY_COUNT   4//5
#define DOCK_LIGHT_TOUCH_THROD  30

BOOLEAN touch_bump_onoff=FALSE;
BOOLEAN tb_on_sig=FALSE;
static u32 front_time_stp=0;

/****************************************************************
*Function   :  turn_off_touch_bump
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  关闭light touch bump功能
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void turn_off_touch_bump()
{
  if(touch_bump_onoff==FALSE)
  {
  }
  touch_bump_onoff=TRUE;
}

/****************************************************************
*Function   :  turn_on_touch_bump
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  打开light touch bump功能
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void turn_on_touch_bump()
{
  if(touch_bump_onoff==TRUE)
  {
    front_time_stp=0;
  }
  touch_bump_onoff=FALSE;
}

BumpState touch_bump_state = 0;
BumpState  get_touch_bump_state(void)
{
	return touch_bump_state;
}

/****************************************************************
*Function   :  touch_bump_run
*Author     :  lyy
*Date       :  2017.4.20
*Description:  计算是否产生lt bump
*CallBy     :  提高给SDK调用，返回结果
*Input      :  无
*Output     :  无
*Return     :  return value:
            0: no bump
            1: left bump
            2: right bump
            3: center bump
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
int touch_bump_run(void)
{
  s16 value_cl;
  u16 value_cr;
  static s16 cl_array[2];
  static s16 cr_array[2];

  s16 value_fl;
  s16 value_fr;
  s16 value_center;
  u32 time_now;
  static u16 value_max=0;
  static u8 decay_count=0;
  static s16 c_array[2];


  static s16 cl_value_max=0;
  static u8 cl_decay_count=0;
  static u32 cl_time_stp=0;

  static s16 cr_value_max=0;
  static u8 cr_decay_count=0;
  static u32 cr_time_stp=0;

  int lt_bump=0;

  touch_bump_state = 0;

  int TOTCH_BUMP_TIME_DELAY = 900;

  if(touch_bump_onoff)
  {
     return 0;
  }
  
  
  value_fl = robot_signal_distance(LT_FRONTLEFT)>>TOUCH_RIGHT_MOVE;
  value_fr = robot_signal_distance(LT_FRONTRIGHT)>>TOUCH_RIGHT_MOVE;

  time_now = timer_ms();

  {
    value_cl = robot_signal_distance(LT_CENTERLEFT)>>TOUCH_RIGHT_MOVE;
    value_cr = robot_signal_distance(LT_CENTERRIGHT)>>TOUCH_RIGHT_MOVE;


    value_cl=mid_filter(cl_array, value_cl);
    value_cr=mid_filter(cr_array, value_cr);
    
    
#ifdef USE_LEFT_RIGHT_LT //用上左右两边的lt 做判断
    {
      value_l=robot_signal_distance(LT_LEFT)>>TOUCH_RIGHT_MOVE;
      value_r=robot_signal_distance(LT_RIGHT)>>TOUCH_RIGHT_MOVE;
      if(!(wall_follow_is_running()))
      {
        if(value_l > lt_max_light_value*3)
        {
          value_cl += lt_max_light_value;
        }
        if(value_r > lt_max_light_value*3)
        {
          value_cr += lt_max_light_value;
        }
      }
    }
#endif
    
  }

  value_center = (value_fl + value_fr)>>1;

  value_center=mid_filter(c_array, value_center);

  //process the front touch bump
  if((value_center >= UI_SET_LT_BUMP_THROD) && (timer_elapsed(front_time_stp)>TOTCH_BUMP_TIME_DELAY))
  {
    //printf("value_center = %d\r\n",value_center);
    if((value_max < value_center) && (value_center<UI_SET_LT_MAX_LIGHT_VALUE))
    {
      value_max = value_center;
      decay_count=0;
    }
    else if(((value_max*7/8) > value_center) || (value_center>=UI_SET_LT_MAX_LIGHT_VALUE))
    {
      decay_count++;
      //printf("decay_count = %d\r\n",decay_count,value_max);
      if(decay_count==LIGHT_BUMP_DELAY_COUNT)
      {
        if(touch_bump_onoff==FALSE)
        {
            lt_bump=3;
            touch_bump_state |= BUMP_FRONT_CENTER;
        }
        front_time_stp = timer_ms();   //this is to delete multiple trigger
      }
    }
  }
  else
  {
    value_max=value_center;
    decay_count=0;
    tb_on_sig=FALSE;
  }

  {
    time_now = timer_ms();

    //if((time_now-front_time_stp)>TOTCH_BUMP_TIME_DELAY)   //when the front is in trigger, don't trigger the other touch
    {
      //center left
      if((value_cl >= UI_SET_LT_BUMP_LEFT_RIGHT_THROD) && (timer_elapsed(cl_time_stp)>TOTCH_BUMP_TIME_DELAY))
      {
        //printf("value_cl = %d\r\n",value_cl);
        if(((cl_value_max < value_cl)) && (value_cl<UI_SET_LT_MAX_LIGHT_VALUE))// || ((cl_value_max + cl_value_max>>3) < value_cl))
        {
          cl_value_max = value_cl;
          cl_decay_count=0;
        }
        else if((cl_value_max*7/8 > value_cl) || ((value_cl>=UI_SET_LT_MAX_LIGHT_VALUE)))
        {
          cl_decay_count++;
          if((cl_decay_count==(LIGHT_BUMP_DELAY_COUNT+3)))// || (value_l>=DOCK_LIGHT_TOUCH_THROD)) )
          {
            if(touch_bump_onoff==FALSE)
            {
              lt_bump |= 1;
              touch_bump_state |= BUMP_FRONT_LEFT;
            }
            cl_time_stp = timer_ms();   //this is to delete multiple trigger
          }
        }
        else
        {
          cl_value_max=value_cl;
          cl_decay_count=0;
        }
      }
      else
      {
        cl_value_max=value_cl;
        cl_decay_count=0;
      }
      //center right
      time_now = timer_ms();
      //if((time_now-cl_time_stp)>TOTCH_BUMP_TIME_DELAY)   //to make sure there is only one touch bump is trigger
      {
        if((value_cr >= UI_SET_LT_BUMP_LEFT_RIGHT_THROD) && (timer_elapsed(cr_time_stp)>TOTCH_BUMP_TIME_DELAY))
        {
         // printf("value_cr = %d\r\n",value_cr);
          if(((cr_value_max <= value_cr)) && (value_cr<UI_SET_LT_MAX_LIGHT_VALUE))// || ((cr_value_max + cr_value_max>>3) <= value_cr))
          {
            cr_value_max = value_cr;
            cr_decay_count=0;
          }
          else if(((cr_value_max*7/8)>value_cr) || ((value_cr >= UI_SET_LT_MAX_LIGHT_VALUE)))
          {
            cr_decay_count++;
            if(cr_decay_count==(LIGHT_BUMP_DELAY_COUNT+3))// || (value_r>=DOCK_LIGHT_TOUCH_THROD)))
            {
              if(touch_bump_onoff==FALSE)
              {
                lt_bump |=2;
                touch_bump_state |= BUMP_FRONT_RIGHT;
              }
              cr_time_stp = timer_ms();   //this is to delete multiple trigger
            }
          }
          else
          {
            cr_value_max=value_cr;
            cr_decay_count=0;
          }
        }
        else
        {
          cr_value_max=value_cr;
          cr_decay_count=0;
        }
      }
    }

  }
  return lt_bump;
}

