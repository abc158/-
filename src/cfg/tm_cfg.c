//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*by liyongyong*/
#include "am_tm_api.h"
#include "am_type.h"
#include "syscall_api.h"
#include "am_device.h"

int gtm0_id;
int gtm1_id;
int gtm2_id;
int capture_id;
int m_pwm_id;
int m_capture_id;
int charge_pwm_id;
int side_pwm_id;

void charger_pwm_handler(int arg)
{
  ;
}

/****************************************************************
*Function   :  sys_tm_cfg
*Author     :  lyy
*Date       :  2017.4.20
*Description:  初始化timer配置
*CallBy     :  任何地方，中断除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sys_tm_cfg(void)
{
  GPTM_TimeBaseInitTypeDef_t gftmInit;
  UserOutInit_t UserOutInit;

  MCTM_TimeBaseInitTypeDef mctmInit;
  MUserOutInit_t MUserOutInit;
  ////////////////////////////gftm pwm///////////////////////////////////////
  //gtm1_id = tm_request(GPTM,GP_TM1_ID);//主刷和边刷
  gtm1_id = open(DEV_GPTM,GP_TM1_ID);

  gftmInit.CounterReload = 1000;
  gftmInit.Prescaler     = 5;  //lqw
  gftmInit.CounterMode   = GPTM_CNT_MODE_UP;
  gftmInit.PSCReloadTime = GPTM_PSC_RLD_IMMEDIATE;
  //tm_gptm_cfg(gtm1_id,&gftmInit);
  ioctl(gtm1_id, TM_GPTM_BASE_INIT, &gftmInit);

  UserOutInit.OutInit.OutputMode        = GPTM_OM_PWM1;//ACTIVE , WHEN  CNTR < CHxCCR
  UserOutInit.OutInit.Control           = GPTM_CHCTL_ENABLE; //enable chx
  UserOutInit.OutInit.Polarity          = GPTM_CHP_NONINVERTED;//h is active
  UserOutInit.OutInit.Compare           = 0; //占空比
  UserOutInit.OutInit.AsymmetricCompare = 0;

  UserOutInit.OutMode.irq         = 0;//GPTM_INT_CH0CC | GPTM_INT_CH1CC | GPTM_INT_CH2CC | GPTM_INT_CH3CC;//中断
  UserOutInit.OutMode.dmaReqMode  = 0;//dma request
  for(int i = 2; i<4; i++)
  {
    UserOutInit.OutInit.Channel  = (GPTM_CH_Enum)i;
    //tm_gptm_output_pwm_cfg(gtm1_id, &UserOutInit);
    ioctl(gtm1_id,TM_GPTM_OUT_MODE_INIT, &UserOutInit);
  }
///////////////////////////////////////////////////

  gtm0_id = open(DEV_GPTM,GP_TM0_ID);//tm_request(GPTM,GP_TM0_ID);

  gftmInit.CounterReload = 1000;
  gftmInit.Prescaler     = 3;//5; //by lqw
  gftmInit.CounterMode   = GPTM_CNT_MODE_UP;
  gftmInit.PSCReloadTime = GPTM_PSC_RLD_IMMEDIATE;
  //tm_gptm_cfg(gtm0_id,&gftmInit);
  ioctl(gtm0_id, TM_GPTM_BASE_INIT, &gftmInit);

  UserOutInit.OutInit.OutputMode        = GPTM_OM_PWM1;//ACTIVE , WHEN  CNTR < CHxCCR
  UserOutInit.OutInit.Control           = GPTM_CHCTL_ENABLE; //enable chx
  UserOutInit.OutInit.Polarity          = GPTM_CHP_NONINVERTED;//h is active
  UserOutInit.OutInit.Compare           = 0; //占空比
  UserOutInit.OutInit.AsymmetricCompare = 0;

  UserOutInit.OutMode.irq         = 0;//GPTM_INT_CH3CC;//中断
  UserOutInit.OutMode.dmaReqMode  = 0;//dma request

  UserOutInit.OutInit.Channel  =(GPTM_CH_Enum)0;
  //tm_gptm_output_pwm_cfg(gtm0_id, &UserOutInit);
  ioctl(gtm0_id,TM_GPTM_OUT_MODE_INIT, &UserOutInit);
////////////////////////////gptm capture////////////////////////////////////////

  gtm2_id = open(DEV_GPTM,GP_TM0_ID);//tm_request(GPTM,GP_TM2_ID);
  gftmInit.CounterReload = 270;
  gftmInit.Prescaler     = 6;
  gftmInit.CounterMode   = GPTM_CNT_MODE_UP;
  gftmInit.PSCReloadTime = GPTM_PSC_RLD_IMMEDIATE;
  //tm_gptm_cfg(gtm2_id,&gftmInit);
  ioctl(gtm2_id, TM_GPTM_BASE_INIT, &gftmInit);

  UserOutInit.OutInit.OutputMode        = GPTM_OM_PWM1;//ACTIVE , WHEN  CNTR < CHxCCR
  UserOutInit.OutInit.Control           = GPTM_CHCTL_ENABLE; //enable chx
  UserOutInit.OutInit.Polarity          = GPTM_CHP_NONINVERTED;//h is active
  UserOutInit.OutInit.Compare           = 0; //占空比
  UserOutInit.OutInit.AsymmetricCompare = 0;

  UserOutInit.OutMode.irq         = 0;//GPTM_INT_CH3CC;//中断
  UserOutInit.OutMode.dmaReqMode  = 0;//dma request

  //UserOutInit.OutInit.Channel  = 0;
  //tm_gptm_output_pwm_cfg(gtm2_id, &UserOutInit);
  for(int i = 0; i<4; i++)
  {
    UserOutInit.OutInit.Channel  = (GPTM_CH_Enum)i;
    //tm_gptm_output_pwm_cfg(gtm2_id, &UserOutInit);
    ioctl(gtm2_id,TM_GPTM_OUT_MODE_INIT, &UserOutInit);
  }


///////////////////////////////mctm pwm///////////////////////////////////
  charge_pwm_id = open(DEV_MCTM,MC_TM2_ID);//tm_request(MCTM,MC_TM2_ID);

  mctmInit.CounterReload = 720;
  mctmInit.Prescaler     = 0;
  mctmInit.CounterMode   = MCTM_CNT_MODE_UP;
  mctmInit.PSCReloadTime = MCTM_PSC_RLD_IMMEDIATE;
  //tm_mctm_cfg(charge_pwm_id,&mctmInit);
  ioctl(charge_pwm_id, TM_MCTM_BASE_INIT, &mctmInit);

  MUserOutInit.OutInit.OutputMode = MCTM_OM_PWM2;
  MUserOutInit.OutInit.Control    = MCTM_CHCTL_ENABLE;
  MUserOutInit.OutInit.ControlN   = MCTM_CHCTL_ENABLE;
  MUserOutInit.OutInit.Polarity   = MCTM_CHP_NONINVERTED;
  MUserOutInit.OutInit.PolarityN  = MCTM_CHP_NONINVERTED;
  MUserOutInit.OutInit.IdleState  = MCTM_OIS_LOW;
  MUserOutInit.OutInit.IdleStateN = MCTM_OIS_HIGH;
  MUserOutInit.OutInit.Compare    = 0;
  MUserOutInit.OutMode.irq        = 0;
  MUserOutInit.OutMode.dmaReqMode = 0;

  MUserOutInit.OutInit.Channel  = (MCTM_CH_Enum)1;
  //tm_mctm_output_pwm_cfg(charge_pwm_id, &MUserOutInit);
  ioctl(charge_pwm_id, TM_MCTM_OUT_MODE_INIT, &MUserOutInit);
  request_irq(charge_pwm_id,(long)charger_pwm_handler,0);
}

/****************************************************************
*Function   :  sys_tm_close
*Author     :  lyy
*Date       :  2017.4.20
*Description:  关闭各路timer
*CallBy     :  任何地方，中断除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sys_tm_close()
{
  close(gtm0_id);
  close(gtm1_id);
  close(gtm2_id);
  close(side_pwm_id);
  close(charge_pwm_id);
}

/****************************************************************
*Function   :  sys_set_pwm_out_duty
*Author     :  lyy
*Date       :  2017.4.20
*Description:  配置各路timer的pwm输出
*CallBy     :  任何地方，中断除外
*Input      :  参数
*              channel：参考各路pwm的宏定义，如SIDE_BRUSH_PWM_CHANNEL，MAIN_BRUSH_PWM_CHANNEL等等
*              Duty   ：pwm占空比值，各路pwm值的范围按照timer配置决定
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sys_set_pwm_out_duty(s16 channel , s16 Duty)
{
  int id = -1;

  s16 Dutyfactor = Duty;
  switch(channel)
  {
  case SIDE_BRUSH_PWM_CHANNEL:
    id = gtm1_id;

    ioctl(id,GPTM_CHAN3_SET_DUTY,(void *)&Dutyfactor);
    break;
  case MAIN_BRUSH_PWM_CHANNEL:
    id = gtm1_id;

    ioctl(id,GPTM_CHAN2_SET_DUTY,(void *)&Dutyfactor);
    break;
  case BIN_LEFT_PWM_CHANNEL:
    id = gtm1_id;

    ioctl(id,GPTM_CHAN0_SET_DUTY,(void *)&Dutyfactor);
    break;
  case BIN_RIGHT_PWM_CHANNEL:
    id = gtm1_id;

    ioctl(id,GPTM_CHAN1_SET_DUTY,(void *)&Dutyfactor);
    break;
  case VACUUM_PWM_CHANNEL:
    id = gtm0_id;

    ioctl(id,GPTM_CHAN0_SET_DUTY,(void *)&Dutyfactor);
    break;
  case CHARGER_PWM_CHANNEL:
    id = charge_pwm_id;

    //tm_mctm_output_duty_set(id,chan,Dutyfactor);
    ioctl(id,MCTM_CHAN1_SET_DUTY,(void *)&Dutyfactor);
	break;
  case HOU2_IR_SEND_PWM_CHANNEL:
    id = gtm2_id;

    break;
  case HOU1_IR_SEND_PWM_CHANNEL:
    id = gtm2_id;

    break;
  case QIAN1_IR_SEND_PWM_CHANNEL:
    id = gtm2_id;

    break;
  case QIAN2_IR_SEND_PWM_CHANNEL:
    id = gtm2_id;

    return;
  default:
    id = -1;
    break;
  }
}


