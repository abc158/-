#include "robot_univrsal_wheel.h"
#include "am_exti_cfg.h"
#include "syscall_api.h"
#include "am_device.h"
#include "sensor/sensor.h"


#define AM_IO_STASIS_IN GPIOB(12)
#define STASIS_ON_COUNT 3

AM_Pose stasis_last_pos;
u32 stasis_count=0;
u32 stasis_test=0;

static int statsis_id = -1;
static BOOLEAN  stasis_throd_bool=FALSE;
static uint8_t  begin_monitor = 0;


//提供给SDK底层使用的
int universal_wheel_count(void)
{
  return stasis_count;
}

//中断处理函数
static void robot_universal_wheel_monitor(u8 arg)
{
  arg = arg;
  robot_pos_get(&stasis_last_pos);
  stasis_count++;
  stasis_test++;
  begin_monitor = (begin_monitor<STASIS_ON_COUNT*2)? (begin_monitor+1):STASIS_ON_COUNT*2;
}

/****************************************************************
*Function   :  robot_stasis_test_clear
*Author     :  lyy    
*Date       :  2017.6.27
*Description:  测试万向轮时，使用。 清除万向轮转动的数据
*CallBy     :  测试模式下
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void robot_stasis_test_clear(void)
{
  stasis_test = 0;
}

/****************************************************************
*Function   :  robot_stasis_test_get
*Author     :  lyy    
*Date       :  2017.6.27
*Description:  获取万向轮转动的次数
*CallBy     :  测试模式下
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回：万向轮转动的次数
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u32 robot_stasis_test_get(void)
{
  return stasis_test;
}

void robot_universal_wheel_init(void)
{
   stasis_count  = 0;
   stasis_test   = 0;
   begin_monitor = 0; 
}

void set_stasis_high_throd()
{
  stasis_throd_bool=TRUE;
}

void set_stasis_normal()
{
  stasis_throd_bool=TRUE;
}

void turn_off_stasis()
{
    set_slip_high_throd(2.2f);
    stasis_throd_bool=TRUE;

}

void turn_on_stasis()
{
  set_slip_normal();
  stasis_throd_bool=FALSE;
}
/////////////////////////////////////////////////
/*
return
0: 表示没有万向轮bump
1：表示有万向轮bump
*/
int universal_wheel_monitor(void *arg)
{
    static float stasis_lenth=STASIS_TRIGGER_NORMAL_LENTH;
    static u32 stasis_time_stp;
    static u8  stasis_count_local=0;
    static u32 stasis_count_ex;
    static am_real last_stasis_dist_from;
    static float stasis_cof=1.0f;
    static uint8_t stasis_run_forward_check_normal_count=21;
    static u32 forward_stasis;
    u8  have_set_stasis_cof;
    s16 c_lefts,c_rights;
    s16 reported_velocity_left,reported_velocity_right;
    int flag = 0;
    
    //arg = arg;//暂时没有使用，留将来扩展使用，用来给SDK底层和APP交互参数

    get_commanded_speeds(&c_lefts,&c_rights);
    get_motor_speeds(&reported_velocity_left,&reported_velocity_right);
    //printf("%d,%d-%d,%d\r\n",(int)(ticks_to_mm(reported_velocity_left) *1000),(int)(ticks_to_mm(reported_velocity_right) *1000),(int)(ticks_to_mm(c_lefts)*1000),(int)(ticks_to_mm(c_rights)*1000));
    if((ticks_to_mm(reported_velocity_left)>30.0f) && (ticks_to_mm(reported_velocity_right)>30.0f))
    {
      AM_Pose robot_pos;
      am_real dist;
      robot_pos_get(&robot_pos);
      dist = robot_xy_dist(&robot_pos.xy, &stasis_last_pos.xy)*1000;

      //make sure the stasis is working correctly,dat:20170418
      if(stasis_count_ex != stasis_count)
      {
    	  stasis_count_ex=stasis_count;
          if(last_stasis_dist_from>STASIS_RUN_LENTH_IN_MM*5)
          {
            if(stasis_run_forward_check_normal_count<60)
            {
              stasis_run_forward_check_normal_count += 5;
            }
          }
          else
          {
            if(stasis_run_forward_check_normal_count>0)
            {
              stasis_run_forward_check_normal_count--;
            }
          }
          
          if(stasis_run_forward_check_normal_count>14)
          {
            printf("stasis over,%d\r\n",stasis_run_forward_check_normal_count);
            have_set_stasis_cof = 1;
            stasis_cof=6.0f;
          }
          else
          {
            have_set_stasis_cof = 0;
            stasis_cof=1.0f;
          }

      }


      if(!have_set_stasis_cof)
      {
        if( (robot_signal_distance(LT_FRONTLEFT) < 40) &&  (robot_signal_distance(LT_FRONTRIGHT) < 40) ) //这个参数根据实际调整
        {
          stasis_cof = 6.0f;
        }  
        else
        {
          stasis_cof = 1.0f;  
        }
      }   
      
      last_stasis_dist_from = dist;
      
      if(begin_monitor>STASIS_ON_COUNT)
      {
        if(((dist > stasis_lenth*stasis_cof) && (stasis_throd_bool==FALSE)) ||  \
            ((dist > STASIS_SET_HIGH_THROD*stasis_cof) && (stasis_throd_bool==TRUE)))
        {
          if(timer_elapsed(stasis_time_stp)<6000)
          {
            stasis_count_local++;
          }
          else
          {
            if(stasis_count_local>0)
            {
            	stasis_count_local--;
            }
          }
          
          if(stasis_count_local>3)
          {
            stasis_lenth=STASIS_ALWAYS_SET_THROD;
            stasis_count_local=3;
            printf("always stasis!!!!!");
          }
          else
          {
            stasis_lenth=STASIS_TRIGGER_NORMAL_LENTH;
          }
          
          printf("stasis %d %d (%d,%d)!\r\n",(int)(dist*100),(int)(stasis_lenth*100),c_lefts,c_rights);
          
          if((c_rights>0) && (c_lefts > 0) && !wall_follow_is_running())//if forward
          {
            if(!forward_stasis)
            {
              forward_stasis = timer_ms();
            }
            else
            {
              if( timer_elapsed(forward_stasis) < 4000 )//如果4000ms内还有，则一般是正前方撞到东西
              {
                flag = 1;
              }
              forward_stasis = 0;
            }            
          }

          robot_pos_get(&stasis_last_pos);
          stasis_time_stp=timer_ms();
        }
      }
      else
      {
        robot_pos_get(&stasis_last_pos);
      }
    }
    else if((abs(reported_velocity_left)>30) || (abs(reported_velocity_right)>30))
    {
      if(begin_monitor>STASIS_ON_COUNT)
      {
        AM_Pose robot_pos;
        am_real dist;
        robot_pos_get(&robot_pos);
        dist = robot_xy_dist(&robot_pos.xy, &stasis_last_pos.xy)*1000;
        if(((dist > STASIS_TRIGGER_HIGH_LENTH*stasis_cof) && (stasis_throd_bool==FALSE)) || \
             ((dist > STASIS_SET_HIGH_THROD*stasis_cof) && (stasis_throd_bool==TRUE)))
        {
          printf("stasis2 %d!\r\n",(int)(dist*100));

          if((c_rights>0) && (c_lefts > 0) && !wall_follow_is_running())//if forward
          {
            if(!forward_stasis)
            {
              forward_stasis = timer_ms();
            }
            else
            {
              if( timer_elapsed(forward_stasis) < 4000 )//如果4000ms内还有，则一般是正前方撞到东西
              {
                flag = 1;
              }
              forward_stasis = 0;
            }            
          }

          robot_pos_get(&stasis_last_pos);
          stasis_time_stp=timer_ms();
        }
      }
    }
    else
    {
      begin_monitor = 0;
      robot_pos_get(&stasis_last_pos);
    }
    return flag;
}


void  robot_universl_wheel_init(void)
{
  exti_init_t exti_user_init;

  statsis_id = open(DEV_EXTI,AM_IO_STASIS_IN);
  exti_user_init.IntEn       = ENABLE;
  exti_user_init.Int_type    = EXTI_BOTH_EDGE;
  exti_user_init.WakeupirqEn = DISABLE;
  exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
  request_irq(statsis_id,(long)robot_universal_wheel_monitor,0);
  ioctl(statsis_id,EXTI_INIT,&exti_user_init);
}

void  robot_universl_wheel_exit(void)
{
  close(statsis_id);
}
