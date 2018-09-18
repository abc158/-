//----------------------------------------------------------------------------
//  Copyright (C) 2001-2011, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.
//----------------------------------------------------------------------------
/*edit by lyy*/
#include <am_type.h>
#include <charge/charge.h>
#include <am_tm_api.h>
#include <syscall_api.h>
#include "robot_suction.h"

#define ROBOT_HAS_VAC_POWER_CONTROL  //if  has vac power 

BOOLEAN suction_off;
s16   cur_duty;
s16   pwm_voltage; 


/****************************************************************
*Function   :  get_suction_current
*Author     :  lyy
*Date       :  2017.4.20
*Description:  真空电流的adc值获取
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回 adc值
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u16 get_suction_current(void)
{
  // need to do....
  return get_adc_chan_val(ADC_CHAN_VAC_CURENT);
}

/****************************************************************
*Function   :  update_suction_duty
*Author     :  lyy
*Date       :  2017.4.20
*Description:  pwm值更新到timer
*CallBy     :  
*Input      :  参数
*              duty：占空比
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void update_suction_duty(int16_t duty)
{
#ifdef USE_BLDC_PWM_CONTROL
static am_uint32 vac_zero_stp=0;
static am_boolean vac_zero_sig=FALSE;
  if(duty == 0 && ((vac_zero_sig == FALSE && timer_elapsed(vac_zero_stp)>1000) || (vac_zero_sig==TRUE)))
  {
    vac_zero_stp = timer_ms();
    vac_zero_sig = TRUE;
    sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,duty);
  }
  else
  {
    if(vac_zero_sig == TRUE && timer_elapsed(vac_zero_stp)<5000)
    {
      sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,1001);
    }
    else
    {
      vac_zero_sig = FALSE;
      sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,duty);
    }
  }
#else
    sys_set_pwm_out_duty(VACUUM_PWM_CHANNEL,duty);
  
#ifdef ROBOT_HAS_VAC_POWER_CONTROL
  // printf("update duty=%d\r\n",duty);
   gpio_set_value(AM_IO_VAC_POWER, ((duty!=0)?1:0));
#endif
   
#endif
}

/****************************************************************
*Function   :  robot_is_suction_on
*Author     :  lyy
*Date       :  2017.4.20
*Description:  判断真空是否有pwm输出
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回 1:有  0：无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
BOOLEAN robot_is_suction_on(void)
{
    return (cur_duty != 0);
}

/****************************************************************
*Function   :  robot_suction_ctrl
*Author     :  lyy
*Date       :  2017.4.20
*Description:  使能真空pwm输出
*CallBy     :  
*Input      :  参数
*              en:  1:使能   0：不使能
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_suction_ctrl(BOOLEAN en)
{
    suction_off = en;
}

/****************************************************************
*Function   :  suction_is_maybe_stall
*Author     :  lyy
*Date       :  2017.4.20
*Description:  判断真空可能被卡住
*CallBy     :  
*Input      :  参数
*              current ： 卡住电流的阈值
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 suction_is_maybe_stall(u16 current)
{  
    return ((get_suction_current() > current) && (suction_off == 0));
}

/****************************************************************
*Function   :  robot_suction_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  真空pwm 动态更新
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
void robot_suction_update(void)
{ 
    if(suction_off == FALSE)
    {
        cur_duty = calc_cur_duty(pwm_voltage);
        update_suction_duty(cur_duty);
    }
    else
    {
        update_suction_duty(0);
    }   
}


/****************************************************************
*Function   :  robot_is_maybe_not_bustbin
*Author     :  lyy
*Date       :  2017.4.20
*Description:  用真空检测垃圾箱
*CallBy     :  
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回 1：垃圾箱被拔出，  0：垃圾箱状态正常
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u8 robot_is_maybe_not_bustbin(void)
{
  return 0;
}

/****************************************************************
*Function   :  robot_suction_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  真空变量等等初始化
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
void robot_suction_init(void)
{
    suction_off = 1;
    cur_duty = 0;
    pwm_voltage  = 0;
   
    robot_suction_adjust_set(VACUUM_NORMAL_VOLTAGE);  

}

/****************************************************************
*Function   :  robot_suction_vols_set
*Author     :  lyy
*Date       :  2017.4.20
*Description:  真空pwm开关
*CallBy     :  
*Input      :  0：关闭pwm   >0:打开pwm
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
extern U16 wetmop_detect(void);
void robot_suction_vols_set(u16 val)
{
    if(val > 0)
      suction_off = 0;
    else
      suction_off = 1;  
	if(wetmop_detect())
	  suction_off = 1;
}

/****************************************************************
*Function   :  robot_suction_adjust_set
*Description:  真空pwm设置
*Input      :  参数
*              val：adc值， 已电池电压为参考点，
*             比如电池电压满伏是16.8 = 3268. 
*             如果要百分百输出，则adc = 3628.
*Output     :  无
*Return     :  无
*Others     :  by ljh
******************************************************************/
void robot_suction_adjust_set(u16 val)
{
    pwm_voltage = val;
}