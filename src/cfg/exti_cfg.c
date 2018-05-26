//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*by liyongyong*/
#include "am_exti_cfg.h"
#include "syscall_api.h"
#include "am_device.h"

/*缓存唤醒原的外部中断id号*/
#define ID_NUM 8
static u32 wakeup_id[ID_NUM]={0};

/****************************************************************
*Function   :  wake_handler
*Author     :  lyy
*Date       :  2017.4.20
*Description:  外部中断唤醒处理函数
*CallBy     :  任何地方
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void wake_handler(void)
{ 
  ;
}

/****************************************************************
*Function   :  exit_wakeup_cfg
*Author     :  lyy
*Date       :  2017.4.20
*Description:  清除外部中断
*CallBy     :  任何地方，中断除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
int16_t exit_wakeup_cfg(void)
{
    int i = 0;
    for(i = 0;i<ID_NUM;i++)
    {
      if(wakeup_id[i]!=0)
      {
        close(wakeup_id[i]);
        wakeup_id[i] = 0;
      }
    }
  
    return 0;
}



/****************************************************************
*Function   :  wakeup_pin_cfg
*Author     :  lyy
*Date       :  2017.4.20
*Description:  配置外部中断
*CallBy     :  任何地方，中断除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
s16 wakeup_pin_cfg(void)
{
    exti_init_t exti_user_init;
    u32 id;
    u8 i = 0;

    exti_user_init.IntEn       = ENABLE;
    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
    exti_user_init.WakeupirqEn = ENABLE;
    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;

    exti_user_init.Wakeup_type = EXTI_WAKEUP_HIGH_LEVEL;
    
    id = open(DEV_EXTI,AM_IO_MC_KEY_WAKE_ON);
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;

    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
    id = open(DEV_EXTI,AM_IO_IR_REC_LEFT);
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;

    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
    id = open(DEV_EXTI,AM_IO_IR_REC_RIGHT);
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;

    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
    id = open(DEV_EXTI,AM_IO_IR_REC_CENTER_LEFT);
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;

    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
    id = open(DEV_EXTI,AM_IO_IR_REC_CENTER_RIGHT);
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;

    
    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
    id = open(DEV_EXTI,AM_IO_IR_REC_BACK_LEFT);
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;
    
    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
    id = open(DEV_EXTI,AM_IO_IR_REC_BACK_RIGHT);
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;

 
    #if 1
    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    exti_user_init.Int_type    = EXTI_LOW_LEVEL;
    id = open(DEV_EXTI,AM_IO_WIFI_WAKE_UP);
/*
    if(id == -1)
    {
        set_led(4);
    }
    */
    ioctl(id,EXTI_INIT,&exti_user_init);
    request_irq(id,(long)wake_handler,0);
    wakeup_id[i++] = id;
    #endif
//    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
//    exti_user_init.Int_type    = EXTI_POSITIVE_EDGE;
//    id = open(DEV_EXTI,IR_OMNI_PIN);//sys_exti_request(IR_OMNI_PIN);
//    ioctl(id,EXTI_INIT,&exti_user_init);
//    request_irq(id,(long)wake_handler,0);
//    wakeup_id[i++] = id;


//    if(ui_interface_get_sleep_level() != CHARGING_SLEEP)
//    {
//      exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
//    }
//    else
//    {
//      if(charging_jack())//如果jack 充电，则把io配置相反，则当机器移开充电座时唤醒
//      {
//        exti_user_init.Wakeup_type = EXTI_WAKEUP_HIGH_LEVEL;
//      }
//      else
//      {
//        exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
//      }
//    }
//    exti_user_init.Int_type = EXTI_POSITIVE_EDGE ;
//    id =  open(DEV_EXTI,IO_CHARGER_HIGH);
//    ioctl(id,EXTI_INIT,&exti_user_init);
//    request_irq(id,(long)from_jack_wake_handler,0);
//    wakeup_id[i++] = id;


//    if(ui_interface_get_sleep_level() != CHARGING_SLEEP)
//    {
//      exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
//    }
//    else
//    {
//      if(charging_dock())//如果dock 充电，则把io配置相反，则当机器移开充电座时唤醒
//      {
//        exti_user_init.Wakeup_type = EXTI_WAKEUP_HIGH_LEVEL;
//      }
//      else
//      {
//        exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
//      }
//    }
//    exti_user_init.Int_type = EXTI_NEGATIVE_EDGE;
//    id = sys_exti_request(hal_io_index(AM_IO_CHARGER_LOW));
//    exti_init(id,&exti_user_init);
//    sys_exti_register_irq_handler(id,(long)from_dock_wake_handler,0);
//    wakeup_id[i++] = id;

//    exti_user_init.Int_type = EXTI_NEGATIVE_EDGE;
//    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
//
//    id = sys_exti_request(BUTTON_SPOT);
//    exti_init(id,&exti_user_init);
//    sys_exti_register_irq_handler(id,(long)from_ui_wake_handler,0);
//    wakeup_id[i++] = id;
//
//    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
//    id = sys_exti_request(BUTTON_DOCK);
//    exti_init(id,&exti_user_init);
//    sys_exti_register_irq_handler(id,(long)from_ui_dock_handler,0);
//    wakeup_id[i++] = id;

    return 0;
}
