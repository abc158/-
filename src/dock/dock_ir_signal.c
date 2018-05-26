//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "am_type.h"
#include "am_robot_type.h"
#include "syscall_api.h"
#include "am_tm_api.h"
#include "docking-new.h"
#include "dock_ir_signal.h"

#ifdef IR_WIRELESS
extern void sys_set_pwm_out_duty(s16 channel , s16 Dutyfactor);
void ir_send_task_config(bool cleaning_dock_mode);

bool  close_yuanpao_ir=FALSE;
bool  has_close_yuanpao_ir=FALSE;////还没关闭
bool  yuanpao_ir_pre_state=FALSE;
bool  close_yuanpao_ir_pre=FALSE;

bool  close_center_ir=FALSE;
bool  has_close_center_ir=FALSE;////还没关闭
bool  center_ir_pre_state=FALSE;
bool  close_center_ir_pre=FALSE;


bool  strong_week=TRUE;
bool  center_pre_strong_week=TRUE;

bool  ir_send_state=FALSE;
bool  ir_send_pre_state=FALSE;
bool  yuanpao_strong_week=TRUE;
bool  yuanpao_pre_strong_week=TRUE;
bool  ir_signal_config_finished=FALSE;

bool  ir_sending_mode=TRUE;//TURE: cleaning mode  FALSE: docking mode
static u8 ir_time_ms_tick;
static u8 ir_send_flag=0;
extern void system_set_pwm_out_duty(s16 channel , s16 Dutyfactor);
void ir_signal_off(void);

void  set_ir_sending_state(bool finish);
bool  get_ir_sending_state(void);
void  set_ir_sending_pre_state(bool finish);
bool  get_ir_sending_pre_state(void);
void  set_yuanpao_pre_strong_week_signal(bool close);
bool get_yuanpao_pre_strong_week_signal(void);
bool  get_center_pre_strong_week_signal(void);
void  set_center_pre_strong_week_signal(bool close);
void ir_send_dynamic_config(bool close_yuanpao, bool close_center, bool yuanpao_strong, bool center_strong);
void restore_pre_ir_send_config(void);
void set_ir_signal_config_finished(bool finish);
bool get_ir_signal_config_finished(void );
#define IR_SEND_TIME 50


void set_ir_signal_config_finished(bool finish)
{
  ir_signal_config_finished=finish;
}
bool get_ir_signal_config_finished(void)
{
  return ir_signal_config_finished;
}


void  set_ir_sending_state(bool finish)
{
  set_ir_sending_pre_state(ir_send_state);
  ir_send_state=finish;
}
bool  get_ir_sending_state(void)
{
  return ir_send_state;
}

void  set_ir_sending_pre_state(bool finish)
{
  ir_send_pre_state=finish;
}
bool  get_ir_sending_pre_state(void)
{
  return ir_send_pre_state;
}

void  set_ir_sending_mode(bool close)
{
  ir_sending_mode=close;
}
bool  get_ir_sending_mode(void)
{
  return ir_sending_mode;
}

void  set_yuanpao_on_pre_stat(bool close)
{
  yuanpao_ir_pre_state=close;
}
bool  get_yuanpao_on_pre_stat(void)
{
  return yuanpao_ir_pre_state;
}
bool  get_yuanpao_on_current_stat(void)
{
  return close_yuanpao_ir;
}

void  set_yuanpao_pre_strong_week_signal(bool close)
{
  yuanpao_pre_strong_week=close;
}

bool get_yuanpao_strong_week_signal(void)
{
  return yuanpao_strong_week;
}
bool get_yuanpao_pre_strong_week_signal(void)
{
  return yuanpao_pre_strong_week;
}

void  set_center_on_pre_stat(bool close)
{
  center_ir_pre_state=close;
}
bool  get_center_on_pre_stat(void)
{
  return center_ir_pre_state;
}
bool  get_center_on_current_stat(void)
{
  return close_center_ir;
}

bool get_center_strong_week_signal(void)
{
  return strong_week;
}
bool  get_center_pre_strong_week_signal(void)
{
  return center_pre_strong_week;
}




void hou1_ir_send_on()
{
  sys_set_pwm_out_duty(HOU1_IR_SEND_PWM_CHANNEL,135);
}
void hou1_ir_send_off()
{
  sys_set_pwm_out_duty(HOU1_IR_SEND_PWM_CHANNEL,0);
}
void hou2_ir_send_on()
{
  sys_set_pwm_out_duty(HOU2_IR_SEND_PWM_CHANNEL,135);
}
void hou2_ir_send_off()
{
  sys_set_pwm_out_duty(HOU2_IR_SEND_PWM_CHANNEL,0);
}
void qian1_ir_send_on()
{
  sys_set_pwm_out_duty(QIAN1_IR_SEND_PWM_CHANNEL,135);
}
void qian1_ir_send_off()
{
  sys_set_pwm_out_duty(QIAN1_IR_SEND_PWM_CHANNEL,0);
}
  //拉低,关38K维持3ms，bit0
void qian2_ir_send_on()//弱信号
{
  sys_set_pwm_out_duty(QIAN2_IR_SEND_PWM_CHANNEL,135);
}
void qian2_ir_send_off()//弱信号
{
  sys_set_pwm_out_duty(QIAN2_IR_SEND_PWM_CHANNEL,0);
}

void ir_signal_off(void)
{
  hou1_ir_send_off();
  hou2_ir_send_off();
  qian1_ir_send_off();
  qian2_ir_send_off();
}
void yuanpao_ir_signal(void)
{
  if(get_yuanpao_on_current_stat())
  {
    hou1_ir_send_off();
    hou2_ir_send_off();
    return;
  }

  if(get_yuanpao_strong_week_signal()==TRUE)
  {
    hou1_ir_send_on();
    hou2_ir_send_off();
  }
  else
  {
    hou1_ir_send_off();
    hou2_ir_send_on();
  }
}

void center_ir_signal(void)
{
  if(get_center_on_current_stat())
  {
    qian1_ir_send_off();
    qian2_ir_send_off();
    return;
  }

  if(get_center_strong_week_signal()==TRUE)
  {
    qian1_ir_send_on();
    qian2_ir_send_off();
  }
  else
  {
    qian1_ir_send_off();
    qian2_ir_send_on();
  }

}


void ir_signal_send(void)
{
  if( !get_ir_signal_config_finished( ))
  {
    ir_signal_off();
    clear_dock_wireless_rx_code();
    return ;
  }
  if(ir_send_flag==1)
  {
    if((ir_time_ms_tick-IR_SEND_TIME)==0)
    {
      //拉高，开38K维持2ms,前导码
      // hou1_ir_send_on();

      yuanpao_ir_signal();
      center_ir_signal();

    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==2)
    {
      //拉低,关38K维持2ms,前导码
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==4)
    {
      //拉高，开38K维持1ms，bit0
      //  hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==5)
    {
      //拉低,关38K维持3ms，bit0
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==8)
    {
      //拉高，开38K维持1ms，bit1
      // hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==9)
    {
      //拉低,关38K维持3ms，bit1
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==12)
    {
      //拉高，开38K维持1ms，bit2
      // hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==13)
    {
      //拉低,关38K维持3ms，bit2
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==16)
    {
      //拉高，开38K维持1ms，bit3
      // hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==17)
    {
      //拉低,关38K维持3ms，bit3
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==20)
    {
      //拉高，开38K维持1ms，bit4
      //hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==21)
    {
      //拉低,关38K维持3ms，bit4
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==24)
    {
      //拉高，开38K维持1ms，bit5
      //hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==25)
    {
      //拉低,关38K维持3ms，bit5
      ir_signal_off();
    }

    else if((ir_time_ms_tick-IR_SEND_TIME)==28)
    {
      //拉高，开38K维持1ms，bit6
      // hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==29)
    {
      //拉低,关38K维持3ms，bit6
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==32)
    {
      //拉高，开38K维持1ms，bit7
      //hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==33)
    {
      //拉低,关38K维持3ms，bit7
      ir_signal_off();
    }
    else if((ir_time_ms_tick-IR_SEND_TIME)==34)
    {
      //拉高，开38K维持1ms，bit7
      // hou1_ir_send_on();
      yuanpao_ir_signal();
      center_ir_signal();
      ir_time_ms_tick=0;
    }
    ir_send_flag=0;
  }
  else
  {
    ir_signal_off();
  }
}

void  close_yuanpao_on(bool close)
{
  set_ir_signal_config_finished(FALSE);
  set_yuanpao_on_pre_stat(close_yuanpao_ir);
  close_yuanpao_ir=close;
}

bool has_yuanpao_off(void)
{
  return has_close_yuanpao_ir;
}
void set_has_yuanpao_off(bool close)
{
  has_close_yuanpao_ir=close;
}

void  close_center_on(bool close)
{
  set_ir_signal_config_finished(FALSE);
  set_yuanpao_on_pre_stat(close_center_ir);
  close_center_ir=close;
}
bool has_center_off(void)
{
  return has_close_center_ir;
}
void set_has_center_off(bool close)
{
  has_close_center_ir=close;
}

void  set_center_pre_strong_week_signal(bool close)
{
  center_pre_strong_week=close;
}
void  set_center_strong_week_signal(bool close)
{
  set_ir_signal_config_finished(FALSE);
  set_center_pre_strong_week_signal(strong_week);
  strong_week=close;
}

void  set_yuanpao_strong_week_signal(bool close)
{
 set_ir_signal_config_finished(FALSE);
  set_yuanpao_pre_strong_week_signal(yuanpao_strong_week);
  yuanpao_strong_week=close;
}

void ir_send_task_config(bool cleaning_dock_mode)
{
    printf("ir_send_task_config\r\n");

  set_ir_signal_config_finished(FALSE);
  set_ir_sending_mode(cleaning_dock_mode);
  if(cleaning_dock_mode==TRUE)//cleaning mode
  {
    close_yuanpao_on(FALSE);
    close_center_on(TRUE);
    set_yuanpao_strong_week_signal(FALSE);
    set_center_strong_week_signal(FALSE);

  }
  else//docking mode
  {
    close_yuanpao_on(FALSE);
    close_center_on(FALSE);
  set_yuanpao_strong_week_signal(TRUE);
   set_center_strong_week_signal(TRUE);
  }
}


void restore_pre_ir_send_config(void)
{
  set_ir_signal_config_finished(FALSE);
  close_yuanpao_on(get_yuanpao_on_pre_stat( ));
  close_center_on(get_center_on_pre_stat( ));
  set_yuanpao_strong_week_signal(get_yuanpao_pre_strong_week_signal());
  set_center_strong_week_signal(get_center_pre_strong_week_signal());
}


void ir_send_dynamic_config(bool close_yuanpao, bool close_center, bool yuanpao_strong, bool center_strong)
{
  set_ir_signal_config_finished(FALSE);
  close_yuanpao_on(close_yuanpao);
  close_center_on(close_center);
  set_yuanpao_strong_week_signal(yuanpao_strong);
  set_center_strong_week_signal(center_strong);
}


void ir_send_task_config_test(void)
{
  set_ir_signal_config_finished(FALSE);
  set_ir_sending_mode(FALSE);

  close_yuanpao_on(FALSE);
  close_center_on(TRUE);
  set_yuanpao_strong_week_signal(FALSE);
  set_center_strong_week_signal(FALSE);

}

static u8 ir_send_cnt=0;
static u8 ir_send_stat_cnt=0;

void ir_send(void)
{

  //static u8 flag=0;
  static bool tmp;

  if(ir_send_cnt>4)
  {
    ir_send_cnt=0;

    if(get_ir_sending_mode()==TRUE)//cleaning
    {
      tmp = get_yuanpao_strong_week_signal()==TRUE ?  FALSE: TRUE;
      set_yuanpao_strong_week_signal(tmp);
    }
  }

  if(get_ir_sending_state()!=get_ir_sending_pre_state())
  {
       ir_send_stat_cnt++;
    if(get_ir_sending_pre_state()==FALSE)
    {
      ir_send_cnt++;


      if(get_yuanpao_on_pre_stat() !=get_yuanpao_on_current_stat()||\
        get_center_on_pre_stat() !=get_center_on_current_stat()||\
          get_yuanpao_pre_strong_week_signal() !=get_yuanpao_strong_week_signal()||\
            get_center_pre_strong_week_signal() !=get_center_strong_week_signal() )
      {
        clear_dock_wireless_rx_code();
      }



      if(get_yuanpao_on_current_stat())
      {
        set_has_yuanpao_off(TRUE);
      }
      else
      {
        set_has_yuanpao_off(FALSE);
      }

      if(get_center_on_current_stat())
      {
        set_has_center_off(TRUE);
      }
      else
      {

        set_has_center_off(FALSE);
      }

      if(ir_send_stat_cnt>4)
      {
        ir_send_stat_cnt=0;
      set_ir_signal_config_finished(TRUE);
      }

    }
  }

  ir_time_ms_tick++;
  if(ir_time_ms_tick>=IR_SEND_TIME)
  {
    ir_send_flag=1;
    set_ir_sending_state(TRUE);
  }
  else
  {
    set_ir_sending_state(FALSE);
  }
  ir_signal_send();
  // hou2_ir_send_off();
}
#endif

