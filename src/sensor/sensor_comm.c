//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include "sensor/sensor.h"
#include "syscall_api.h"
#include "am_date_base.h"

static u8 g_sensor_start_gather;
extern void sensor_handle_touch(void);
extern void sensor_gather_touch(void);
extern void sensor_handle_cliff(void);
extern void sensor_gather_cliff(void);
extern volatile u16 time_4khz_counter_touch;
u16 mid_filter(s16* data_array, u16 new_data)
{
  u16 temp;
  u16 temp_array[3];
  
  temp_array[0]=*data_array;
  temp_array[1]=*(data_array+1);
  temp_array[2]=new_data;
  
  if(temp_array[0]>temp_array[1])
  {
    temp=temp_array[1];
    temp_array[1]=temp_array[0];
    temp_array[0]=temp;
  }
  if(temp_array[1]>temp_array[2])
  {
    temp=temp_array[2];
    temp_array[2]=temp_array[1];
    temp_array[1]=temp;
  }
  if(temp_array[0]>temp_array[1])
  {
    temp=temp_array[1];
    temp_array[1]=temp_array[0];
    temp_array[0]=temp;
  }
  
  *data_array=*(data_array+1);
  *(data_array+1)=new_data;
  return temp_array[1]; 
}


/****************************************************************
*Function   :  robot_sensor_gather_start
*Author     :  lyy
*Date       :  2017.4.20
*Description:  控制采样使能
*CallBy     :  
*Input      :  参数
*              en: 1： 打开   0：关闭
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_sensor_gather_start(u8 en)
{
  g_sensor_start_gather = en;
}

/****************************************************************
*Function   :  robot_sensor_handler
*Author     :  lyy
*Date       :  2017.4.20
*Description:  4k的中断采样和处理过程
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
u8 touch_divide=0x07;
extern u8 light_index_on[2];
extern u8 light_index_off[2];
long robot_sensor_handler(void)
{
  static u8 led_close = 0;
  static u8 lt_cnt = 0;
  static u8 timer_divide=1;
  static U8 devide_flag=0;
	#ifdef SYS_CHECK
  timer2_flag = 0;
	#endif
  if(g_sensor_start_gather)
  {
    led_close = 0;
    sensor_gather_cliff();
    sensor_handle_cliff();
    hal_isr();
    if(dock_is_enable())
    {
      if(devide_flag!=1)
      {
        timer_divide=1;
        touch_divide=0x3f;
        devide_flag=1;
        for(U8 i=0;i<2;i++)
        {
          light_index_on[i]=0;
          light_index_off[i]=0;
        }
        time_4khz_counter_touch=0;
      }
    }
    else 
    {
      if(devide_flag!=2)
      {
        timer_divide=1;
        touch_divide=0x1f;
        devide_flag=2;
        for(U8 i=0;i<2;i++)
        {
          light_index_on[i]=0;
          light_index_off[i]=0;
        }
        time_4khz_counter_touch=0;
      }
    }
    lt_cnt++;
    if (lt_cnt >= timer_divide)
    {
      lt_cnt = 0;
      sensor_gather_touch();
     sensor_handle_touch();
     time_4khz_counter_touch = (time_4khz_counter_touch + 1) & 0xff;

    }   
    
  }
  else
  {
    if(led_close == 0)
    {
      led_close = 1;
      robot_sensor_init();
      robot_close_sensor_led();

    }
    lt_cnt = 0;
    time_4khz_counter_touch = 0;
  }
  return (1);
}

/****************************************************************
*Function   :  robot_ir_detection_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  初始化lt，cliff采样模块，申请一个4k的中断来进行采样。
                所以整个采样和处理过程都是在中断里
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
void robot_ir_detection_init(void)
{
  robot_sensor_init();
  sys_timer_register(HZ_4K,(long)robot_sensor_handler,0);
}
