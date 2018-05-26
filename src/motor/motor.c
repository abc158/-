//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include <am_type.h>
#include <charge/charge.h>
#include <am_tm_api.h>
#include <syscall_api.h>
#include <motor/robot_brush.h>
#include <motor/robot_suction.h>

BOOLEAN initialized; 

/****************************************************************
*Function   :  calc_cur_duty
*Author     :  lyy
*Date       :  2017.4.20
*Description:  把adc值转成要输出的pwm占空比
*CallBy     :  
*Input      :  参数
*              voltage： adc值， 已电池电压为参考点，
*             比如电池电压满伏是16.8 = 3268. 
*             如果要百分百输出，则adc = 3628.
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
int calc_cur_duty(int16_t voltage)
{
  int percent = ((voltage*100)/CHARGING_MAXVOL );
  const u8 limit = 100;

  if(percent > limit)
    percent = limit;  
        
  int pwm = (percent * 10);
  
  return pwm;
}

/****************************************************************
*Function   :  robot_motor_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  真空，主边刷 pwm update
*CallBy     :  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_motor_update()
{
  robot_suction_update();
  robot_brush_update();
}

/****************************************************************
*Function   :  robot_motor_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  真空，主边刷 控制init，启动一个定时器去动态update pwm
*CallBy     :  
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_motor_init(void)
{
    if(!initialized)
    {
        timer_task_register(robot_motor_update, 100, TRUE);
        initialized = 1;
    }
    robot_suction_init();
    robot_brush_init();
}