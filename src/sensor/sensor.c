//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*****************************************************************************
* edit by yongyong.li
*****************************************************************************/
#include "sensor.h"
#include "hal_amicro_gpio.h"
#include "adc_chan.h"
#include "syscall_api.h"
#include "ui-config.h"
#include "sensor/sensor.h"

extern const adc_chan_t adc_chan_table[ADC_CHAN_MAX];
extern const IO_PIN_CFG io_table[HAL_MAX];


/*缓存每次采样的adc值*/
static U16 adcCache[ADC_CHAN_MAX];

volatile u16 time_4khz_counter = 0;
volatile u16 time_4khz_counter_touch = 0;

/*cliff计数采样过程统计的次数*/
volatile u8 cliff_index_on = 0;
volatile u8 cliff_index_off= 0;	

/*lt计数采样过程统计的次数*/
volatile u8 light_index_on[2]={0};
volatile u8 light_index_off[2]={0};

void print_touch(void);
void print_cliff(void);

static u8 useok=0;
static u8 robot_ir_init=0;

static s16 cliff_filter[4];

U8 cliff_hight_flag=0;
 
static uint8_t lt_signal_adjust_enable = 0;
#define USE_NEW_LT_AUTO_ADJUST
#ifdef USE_NEW_LT_AUTO_ADJUST
#define SAME_COUNT  3
#define LT_VALUE_CHANGE  35
#define LT_NEW_LT_THOROD_MAX  3200
#define STRONG_LINGHT_VALUE   7000  //强光下linght touch on off之和的最小值
#define LT_ADJ_LENGTH    60     //调整距离单位mm
#define SPEED_DEVIDE     3      //机器的减速分频比，如果没有设定过默认为3
static s16 signal_offset[6]={0};
static s16 signal_offset_ex[6]={0};
static u8 lt_same_count[6]={0};
static u8 lt_debug_count=0;
static u8 lt_adj_time[6]={0};
static U32 offset_sum[6]={0};
static U16 offset_count[6]={0};
static U16 offset_average[6]={0};
static U32 LT_run_time=16;
static U32 LT_run_time_start=0;
U16 LT_adj_time;
static U16 touch_count=0;
static U8 LT_touch_init_flag=0;
#endif

typedef struct
{
  u8 logic_chan;/*逻辑通道*/
  u8 phy_chan;  /*物理通道*/
}ir_sensor_map_t;

/*cliff , lt 的阈值配置， 变量会在sensor_threshold_update里初始化，on: led灯打开时， off：led关闭时*/
static u16 signal_threshold_on[IR_SENSOR_NUM];
static u16 signal_threshold_off[IR_SENSOR_NUM];

/*逻辑通道和物理通道影响关系结构体表*/
const ir_sensor_map_t remap[IR_SENSOR_NUM]={
  {CLIFF_RIGHT,ADC_CHAN_CLIFF_RIGHT}, 
  {CLIFF_FRONTRIGHT,ADC_CHAN_CLIFF_FRONTLEFT},
  {CLIFF_FRONTLEFT,ADC_CHAN_CLIFF_FRONTLEFT},
  {CLIFF_LEFT,ADC_CHAN_CLIFF_LEFT},
  {CLIFF_REAR_RIGHT,0xFF},
  {CLIFF_REAR_LEFT,0xFF},
  {CLIFF_REV1,0xff},
  {CLIFF_REV2,0xff},  
  {LT_CENTERRIGHT,ADC_CHAN_LT_CENTERRIGHT},
  {LT_FRONTLEFT,ADC_CHAN_LT_FRONTLEFT},
  {LT_RIGHT,ADC_CHAN_LT_RIGHT},
  {LT_LEFT,ADC_CHAN_LT_LEFT},
  {LT_FRONTRIGHT,ADC_CHAN_LT_FRONTLEFT},
  {LT_CENTERLEFT,ADC_CHAN_LT_CENTERLEFT},
  {LT_CENTERLEFT_L,0xff},
  {LT_CENTERRIGHT_L,0xff},
};

/*cliff，lt的结果*/
static u8 signal_result[IR_SENSOR_NUM] = {0};
/*保存最近4次led on 时的adc值*/
static s16 signal_queue_on[IR_SENSOR_NUM][4];
/*保存最近4次led off 时的adc值*/
static s16 signal_queue_off[IR_SENSOR_NUM][4];
/*保存最近4次led on 时的adc值总和*/
 s16 signal_average_on[IR_SENSOR_NUM] ={0};
/*保存最近4次led off 时的adc值总和*/
 s16 signal_average_off[IR_SENSOR_NUM] = {0};
/*保存一个周期 led on 和 off 的采样差值*/
volatile s16 signal_delta[IR_SENSOR_NUM]  = {0};

#define CLIFF_LED_ADC(M)          \
  M( ADC_CHAN_CLIFF_RIGHT)        \
  M( ADC_CHAN_CLIFF_FRONTRIGHT ) \
  M( ADC_CHAN_CLIFF_FRONTLEFT) \
  M( ADC_CHAN_CLIFF_LEFT)

#define LT_LED_ADC(M)          \
  M( ADC_CHAN_LT_CENTERRIGHT)        \
  M( ADC_CHAN_LT_FRONTLEFT ) \
  M( ADC_CHAN_LT_RIGHT)       \
  M( ADC_CHAN_LT_LEFT) \
  M( ADC_CHAN_LT_FRONTRIGHT) \
  M( ADC_CHAN_LT_CENTERLEFT)

#define SAMPLE_ADC(i)           adcCache[i] = (U16)(adcResult_p[adc_chan_table[i].phy_chan]); 
#define SAMPLE_LT_LED()         LT_LED_ADC(SAMPLE_ADC)
#define SAMPLE_CLIFF_LED()      CLIFF_LED_ADC(SAMPLE_ADC)
        
#define SENSOR_LED_ON   1
#define SENSOR_LED_OFF  0

/*lt 控制开关*/
static u8 lt_onoff_swith = 0;
static u8 wf_lt_onoff_swith = 0;
/*cliff 控制开关*/
static u8 cliff_onoff_swith = 0;

/****************************************************************
*Function   :  sensor_threshold_update
*Author     :  lyy
*Date       :  2017.4.20
*Description:  初始化lt，cliff阈值
*CallBy     :  
*Input      :  参数
*              ui_config:  参考 ui_config_t 结构
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void sensor_threshold_update(const ui_local_config_t* ui_config)
{
  //CLIFF ON

  signal_threshold_on[CLIFF_RIGHT]      = ui_config->cliff_threshold.right_on;
  signal_threshold_on[CLIFF_FRONTRIGHT] = ui_config->cliff_threshold.front_right_on;
  signal_threshold_on[CLIFF_FRONTLEFT]  = ui_config->cliff_threshold.front_left_on;  
  signal_threshold_on[CLIFF_LEFT]       = ui_config->cliff_threshold.left_on;
  signal_threshold_on[CLIFF_REAR_RIGHT] = ui_config->cliff_threshold.rear_right_on;
  signal_threshold_on[CLIFF_REAR_LEFT]  = ui_config->cliff_threshold.rear_left_on;
  //CLIFF OFF
  signal_threshold_off[CLIFF_RIGHT]     = ui_config->cliff_threshold.right_off; 
  signal_threshold_off[CLIFF_FRONTRIGHT]= ui_config->cliff_threshold.front_right_off;
  signal_threshold_off[CLIFF_FRONTLEFT] = ui_config->cliff_threshold.front_left_off;
  signal_threshold_off[CLIFF_LEFT]      = ui_config->cliff_threshold.left_off;
  signal_threshold_off[CLIFF_REAR_RIGHT]= ui_config->cliff_threshold.rear_right_off;
  signal_threshold_off[CLIFF_REAR_LEFT] = ui_config->cliff_threshold.rear_left_off;  
  
  //LT ON  
  signal_threshold_on[LT_CENTERRIGHT]  = ui_config->lighttouch_threshold.center_right_on;
  signal_threshold_on[LT_FRONTLEFT]    = ui_config->lighttouch_threshold.front_left_on;
  signal_threshold_on[LT_RIGHT]        = ui_config->lighttouch_threshold.right_on;
  signal_threshold_on[LT_LEFT]         = ui_config->lighttouch_threshold.left_on;
  signal_threshold_on[LT_FRONTRIGHT]   = ui_config->lighttouch_threshold.front_right_on;
  signal_threshold_on[LT_CENTERLEFT]   = ui_config->lighttouch_threshold.center_left_on;
  //LT OFF
  signal_threshold_off[LT_CENTERRIGHT] = ui_config->lighttouch_threshold.center_right_off; 
  signal_threshold_off[LT_FRONTLEFT]   = ui_config->lighttouch_threshold.front_left_off;
  signal_threshold_off[LT_RIGHT]       = ui_config->lighttouch_threshold.right_off;
  signal_threshold_off[LT_LEFT]        = ui_config->lighttouch_threshold.left_off;
  signal_threshold_off[LT_FRONTRIGHT]  = ui_config->lighttouch_threshold.front_right_off;
  signal_threshold_off[LT_CENTERLEFT]  = ui_config->lighttouch_threshold.center_left_off;
}

/****************************************************************
*Function   :  set_cliff_threshold
*Author     :  lyy
*Date       :  2017.4.20
*Description:  设置cliff的阈值
*CallBy     :  
*Input      :  参数
*              chan:  SENSOR_E cliff逻辑通道
*              val :  阈值(adc)
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void set_cliff_threshold(SENSOR_E chan, int val)
{
  signal_threshold_on[chan] = val;
  signal_threshold_off[chan] = val;
}

/****************************************************************
*Function   :  reset_cliff_threshold
*Author     :  lyy
*Date       :  2017.4.20
*Description:  复位cliff阈值
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
void reset_cliff_threshold(void)
{
  sensor_threshold_update(get_local_ui_config());
}

/****************************************************************
*Function   :  sensor_gather
*Author     :  lyy
*Date       :  2017.4.20
*Description:  cliff，lt 的采样流程
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
extern u8 touch_divide;
void sensor_gather_cliff(void)
{
  if(robot_ir_init==0)
    return;
  
  U32 *adcResult_p = (U32 *)ADC_BASE_ADDR;
  int  adcStep = (time_4khz_counter & 0x07);
  switch (adcStep)
  {
  case 0:    
    if(!cliff_onoff_swith)
    {
      gpio_set_value(AM_IO_CLIFF_LED,SENSOR_LED_ON); //
    }
    break;
  case 1: 
    break;
  case 2:
    break;	    
  case 3:  
    SAMPLE_CLIFF_LED();  
    break;
  case 4:
    gpio_set_value(AM_IO_CLIFF_LED,SENSOR_LED_OFF); //   
    break;
  case 5:
    break;
  case 6:   
    SAMPLE_CLIFF_LED();//CLIFF OFF
    break;
  case 7:
    break;
  default:
    break;
  }
}

void sensor_gather_touch(void)
{
  if(robot_ir_init==0)
    return;
  
  U32 *adcResult_p = (U32 *)ADC_BASE_ADDR;
  int  adcStep = (time_4khz_counter_touch & touch_divide);
  switch (adcStep)
  {
  case 0:
    if(!lt_onoff_swith)
    {
      gpio_set_value(AM_IO_LT_CRCL_LED,SENSOR_LED_ON); 
			gpio_set_value(AM_IO_LT_FRFL_LED,SENSOR_LED_ON); 
    }
    if(!wf_lt_onoff_swith)
    gpio_set_value(AM_IO_LT_RL_LED,SENSOR_LED_ON); 
    break;
  case 1: 
		SAMPLE_LT_LED();//lt on 
    gpio_set_value(AM_IO_LT_RL_LED,SENSOR_LED_OFF); 
    gpio_set_value(AM_IO_LT_CRCL_LED,SENSOR_LED_OFF); 
	  gpio_set_value(AM_IO_LT_FRFL_LED,SENSOR_LED_OFF); 
    break;
  case 2:
    break;	    
  case 3: 
    break;
  case 4:
    break;
  case 5:
    break;
  case 6:   
    SAMPLE_LT_LED();  //lt off
    break;
  case 7:
    break;
  default:
    break;
  }
}


/****************************************************************
*Function   :  sensor_handle
*Author     :  lyy
*Date       :  2017.4.20
*Description:  cliff，lt 的采样后，结果处理流程
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
void sensor_handle_cliff(void)
{
  s16 temp = 0;
  if(robot_ir_init==0)
    return;
  
  int  adcStep = time_4khz_counter & 0x07; 
  
  if( cliff_index_off > 3 )
  {
      useok = 1;
  }
  
  switch (adcStep)
  {
  case 0:  
    break;
  case 1: //calc result 
    break;
  case 3:   
    {//cliff
        u8 i = 0;
        if(cliff_index_on >=4)
        {
            cliff_index_on = 0;             
        }
        
        {
           for(i=0;i<=3;i++) 
           {
               signal_average_on[i] = 0;
               signal_queue_on[i][cliff_index_on] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j=0;j<4;j++)
               {
                 signal_average_on[i] += signal_queue_on[i][j]; 
               }               
           }
        }
        cliff_index_on++;   
    }    
    break;
  case 2:  
    break;
  case 6://off
    { 
        u8 i = 0;
        if(cliff_index_off >=4)
        {
            cliff_index_off = 0;             
        }
        
        {
           for(i=0;i<=3;i++)
           {
               signal_average_off[i] = 0;
               signal_queue_off[i][cliff_index_off] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j = 0; j<4; j++)
               {
                 signal_average_off[i] += signal_queue_off[i][j]; 
               }
           }
        }
        cliff_index_off++; 
    }         
    break;
  case 7://calc result
    if(cliff_index_off >= 4 && cliff_index_on >= 4)
    {
    	u8 cliff = 0;
        u8 i = 0;
        s16 temp_s16;
	if(useok==0)
	{
	    break;
	}
        
    	for(i = 0;i<=3;i++)
        {
            temp_s16 = (abs(signal_average_off[i] - signal_average_on[i]));
            if(temp_s16 > cliff_filter[i])
            {
              signal_delta[i]=temp_s16;
            }
            else
            {
              signal_delta[i]=cliff_filter[i];
            }
               
            
            if(((signal_average_off[i] + signal_average_on[i])<2*1024) && (signal_delta[i]  < signal_threshold_off[i]))
            {
              signal_delta[i]=signal_delta[i]<<4;
            }
            

            if(signal_delta[i]  >= signal_threshold_on[i])
            {
                cliff = 0;
            }
            else if(signal_delta[i]  < signal_threshold_off[i])
            {
                cliff = 1;
            }
            else
            {
                cliff = signal_result[i] & 0x1;
            }
            
            cliff_filter[i]=temp_s16;
            
            signal_result[i] = signal_result[i] << 1 ;
            signal_result[i] = signal_result[i] | cliff ;
            signal_result[i] = signal_result[i] & 0xff;
            
        }
        cliff_index_off = 0;
        cliff_index_on  = 0;
    }    
    break;
  case 4:    
    break;
  case 5:
    break;
  }
  /*把lt的结果传送到sdk*/
  robot_lt_update(signal_result);
}
void sensor_handle_touch(void)
{
  s16 temp = 0;
	static U8 go_straight_flag=0;
  if(robot_ir_init==0)
  {
    return;
  }
  
  int  adcStep = time_4khz_counter_touch & touch_divide; 
  
  if( cliff_index_off > 3 )
  {
      useok = 1;
  }
  
  switch (adcStep)
  {
  case 0:  
    break;
  case 1: 
      {//lt
          int i = 0;
          if(light_index_on[0] >=4)
          {
            light_index_on[0] = 0;             
          }          
           for(i=8;i<=13;i++) 
           {
               signal_average_on[i] = 0;
               signal_queue_on[i][light_index_on[0]] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j = 0; j<4; j++)
               {               
                 signal_average_on[i] += signal_queue_on[i][j]; 
               }               
           }
           light_index_on[0]++; 
    }  
    break;
  case 3:     
    break;
  case 2:  
    break;
  case 6://off       
    {//lt
          u8 i = 0;
          if(light_index_off[0] >=4)
          {
            light_index_off[0] = 0;             
          }          
           for(i=8;i<=13;i++) 
           {
               signal_average_off[i] = 0;
               signal_queue_off[i][light_index_off[0]] = (adcCache[remap[i].phy_chan] & 0x00000fff);
               for(int j = 0; j<4; j++)
               {
                 signal_average_off[i] += signal_queue_off[i][j]; 
               }
           }
           light_index_off[0]++; 
    }
  
    break;
  case 7://calc result    
    if(light_index_off[0] >=4 && light_index_on[0]>=4)
    {
    	u8 lt = 0;
        u8 i = 0;
	if(useok==0)
	{
	    break;
	}
#ifdef USE_NEW_LT_AUTO_ADJUST
  int16_t left_speed, right_speed;
  get_motor_speeds(&left_speed,&right_speed);
	left_speed=ticks_to_mm(left_speed);
	right_speed=ticks_to_mm(right_speed);
  if((timer_elapsed(LT_run_time_start))<200)
  {
      LT_run_time=timer_elapsed(LT_run_time_start);
  }
  else
  {
      LT_run_time=16;
  }
  LT_run_time_start=timer_ms();
	
	if((abs(left_speed+right_speed)>(CLEAN_SPEED_MAX*2/SPEED_DEVIDE-20))&&(abs(left_speed-right_speed)<20)&&(wall_follow_is_running()==FALSE))
		{
				go_straight_flag=1;
//判断距离s 本函数运行时间t 当前速度为v  所以调整次数计算为s/v/t  乘以1000是转换为ms
				LT_adj_time=LT_ADJ_LENGTH*1000/right_speed/LT_run_time;
		}	
#endif
    	for(i = 8;i<=13;i++)
        {   
#ifdef USE_NEW_LT_AUTO_ADJUST
//该处理是针对机器本身遮光片的磨损或者进灰导致产生偏差进行校准，
            s16 temp_data=abs(signal_average_off[i] - signal_average_on[i]); 
            if((signal_average_off[i] + signal_average_on[i])<STRONG_LINGHT_VALUE)
            {
                continue;
            }
            if((go_straight_flag==1)&&(temp_data<=LT_NEW_LT_THOROD_MAX))
            {
              if(lt_adj_time[i-8]==0)
              {
                  signal_offset_ex[i-8]=temp_data;
              }
              lt_adj_time[i-8]++;
              if(lt_adj_time[i-8]>LT_adj_time)
              {
             			lt_adj_time[i-8]=0;
                  if((abs(temp_data-signal_offset_ex[i-8])<LT_VALUE_CHANGE))//在调整距离内变化值
                  {
                      lt_same_count[i-8]++;
                  }
									else
									{
											lt_same_count[i-8]=0;
									}
									if(lt_same_count[i-8]>SAME_COUNT)
                  {
                  		lt_same_count[i-8]=0;
                      signal_offset[i-8]=((signal_offset_ex[i-8]>temp_data)? temp_data:signal_offset_ex[i-8]);
											offset_count[i-8]++;
											offset_sum[i-8]+=signal_offset[i-8];
											offset_average[i-8]=offset_sum[i-8]/offset_count[i-8];
											if((offset_average[i-8]<signal_offset[i-8]))
												{
												signal_offset[i-8]=offset_average[i-8];
												}
											if(offset_count[i-8]>1000)
												{
												offset_count[i-8]=1;
												offset_sum[i-8]=signal_offset[i-8];
												}
											lt_debug_count=1;
                  }
              }
                    
            }
            else
            {
                lt_same_count[i-8]=0;
                lt_adj_time[i-8]=0;
            }
						//每次初始化LT后的1.5s，先采一个值当做校准值
						if(LT_touch_init_flag)
						{
							touch_count++;
							//由于for里会循环6次所以乘以一个6
							if(touch_count*LT_run_time<=1500*6)
              //if(((left_speed+right_speed)<20))
								{
									offset_sum[i-8]+=temp_data;
									offset_count[i-8]++;
									signal_offset[i-8]=offset_sum[i-8]/offset_count[i-8];
								}
							else
								{
                    lt_debug_count=1;
									for(U8 i=0;i<6;i++)
									{
										offset_sum[i-8]=0;
										offset_count[i-8]=0;
									}
									LT_touch_init_flag=0;
									touch_count=0;
								}
						}
						signal_delta[i] = (temp_data > signal_offset[i-8])? (temp_data - signal_offset[i-8]):0;	
#else  
            signal_delta[i] = abs(signal_average_off[i] - signal_average_on[i]); 
#endif
            if(signal_delta[i]  >= signal_threshold_on[i])
            {
                lt = 1;//lyy 1--0
                if((i==LT_LEFT)||(i==LT_RIGHT))
									lt = 0;
                //printf("touch: %d %d \r\n", i, signal_delta[i]);
            }
            else if(signal_delta[i]  < signal_threshold_off[i])
            {
                lt = 0;
            }
            else
            {
                lt = signal_result[i] & 0x1;
            }
            signal_result[i] = signal_result[i] << 1 ;
            signal_result[i] = signal_result[i] | lt ;
            signal_result[i] = signal_result[i] & 0xff;
        }   
        
        go_straight_flag=0;
        light_index_off[0] =0;
        light_index_on[0]  =0;
    }
    break;
  case 4:    
    break;
  case 5:
    break;
  }
  /*把lt的结果传送到sdk*/
  robot_lt_update(signal_result);
}

/****************************************************************
*Function   :  robot_close_sensor_led
*Author     :  lyy
*Date       :  2017.4.20
*Description:  闭关led灯，主要是节省功耗
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
void robot_close_sensor_led(void)
{
    gpio_set_value(AM_IO_LT_RL_LED,SENSOR_LED_OFF); //luyanjin   
    gpio_set_value(AM_IO_LT_CRCL_LED,SENSOR_LED_OFF); 
	  gpio_set_value(AM_IO_LT_FRFL_LED,SENSOR_LED_OFF); 
		gpio_set_value(AM_IO_CLIFF_LED,SENSOR_LED_OFF);
 
}


uint16_t hal_isr(void)
{
  if(robot_ir_init==0)
    return 1;
  time_4khz_counter = (time_4khz_counter + 1) & 0xff;
  return (1);
}

s16 robot_rear_lt_distance(u8 ingdex)
{
  return 0;
}

s16 robot_signal_distance(u8 index)
{
  return signal_delta[index];
}

u8 robot_is_cliff(u8 index)
{
  if(signal_result[index])
  	return 1;
  else
  	return 0;
}

u8 robot_is_lighttouch(u8 index)
{
  if(signal_result[index])
  	return 1;
  else
  	return 0;
}

u8 robot_is_rear_cliff(void)
{
  return 0;
}

u8 robot_is_front_cliff(void)
{
  return 0;
}

void set_lighttouch_enable(u8 en)
{
  lt_onoff_swith = en;
}
void set_wf_lighttouch_enable(u8 en)
{
  wf_lt_onoff_swith = en;
}
u8 get_wf_lighttouch_enable(void)
{
  return wf_lt_onoff_swith;
}
void set_cliff_enable(u8 en)
{
  cliff_onoff_swith = en;
}

void reset_lt_auto_offset()
{
#ifdef USE_LT_AUTO_ADJUST
  int i;
  for(i=0;i<6;i++)
  {
    signal_offset[i] = LT_AUTO_ADJUST_THROD;
  }
#endif
}

void robot_sensor_init(void)
{
  int j,i;
  useok = 0;	  
  time_4khz_counter   = 0;  
  cliff_index_on     = 0;
  cliff_index_off    = 0;
  light_index_on[0]  = 0;
  light_index_on[1]  = 0;
  light_index_off[0] = 0;
  light_index_off[1] = 0;
  
  for(i=0;i<IR_SENSOR_NUM;i++)
  {
    signal_result[i]=0;
    signal_average_on[i] =0;
    signal_average_off[i]=0;
    signal_delta[i]=0;
  }

#ifdef USE_NEW_LT_AUTO_ADJUST
	for(i=0;i<6;i++)
		{
		signal_offset[i]=0;
		signal_offset_ex[i]=0;
		lt_same_count[i]=0;
		lt_adj_time[i]=0;
		offset_sum[i]=0;
		offset_count[i]=0;
		offset_average[i]=0;

		}
	LT_adj_time=0;
	lt_debug_count=0;
	LT_touch_init_flag=1;
#endif
  
  for(i=0;i<IR_SENSOR_NUM;i++)
  {
    for(j=0;j<4;j++)
    {
      signal_queue_on[i][j] = 0;
      signal_queue_off[i][j]= 0; 
    }
  }
  reset_lt_auto_offset();
  //reset_cliff_threshold();
  sensor_threshold_update(get_local_ui_config());
  robot_ir_init = 1;
}


void print_touch(void)
{
        static u8 print_touch_cnt = 0;
        
	//robot_sensor_gather_start(1);

        print_touch_cnt++;
        if (print_touch_cnt >= 5)
        {
#ifdef USE_NEW_LT_AUTO_ADJUST
    printf("touch: l=%d,cl=%d,fl=%d,fr=%d,cr=%d,r=%d\r\n", \
            signal_delta[LT_LEFT], signal_delta[LT_CENTERLEFT], signal_delta[LT_FRONTLEFT], \
            signal_delta[LT_FRONTRIGHT], signal_delta[LT_CENTERRIGHT], signal_delta[LT_RIGHT]);
#else
     printf("touch: l=%d,cl=%d,fl=%d,fr=%d,cr=%d,r=%d\r\n", \
            signal_delta[LT_LEFT], signal_delta[LT_CENTERLEFT], signal_delta[LT_FRONTLEFT], \
            signal_delta[LT_FRONTRIGHT], signal_delta[LT_CENTERRIGHT], signal_delta[LT_RIGHT]);
#endif
        print_touch_cnt = 0;
        }
}

void print_cliff(void)
{
  static u8 print_cliff_cnt = 0;
	robot_sensor_gather_start(1);
        
        print_cliff_cnt++;
        if (print_cliff_cnt >= 5)
        {
          //  printf("%d %d\r\n",signal_average_on[CLIFF_FRONTLEFT],signal_average_off[CLIFF_FRONTLEFT]);
        printf("cliff: l=%d,fl=%d,fr=%d,r=%d\r\n",\
         signal_delta[CLIFF_LEFT], signal_delta[CLIFF_FRONTLEFT], signal_delta[CLIFF_FRONTRIGHT], \
         signal_delta[CLIFF_RIGHT]); 
        print_cliff_cnt = 0;
        }
}

void print_debug_lt_auto()
{
#ifdef USE_NEW_LT_AUTO_ADJUST
    static U8 lt_count=0;
    lt_count++;
    if(lt_debug_count>0)
    {
        lt_count=0;
      lt_debug_count=0;
			#if 0
			printf("touch: l=%d cl=%d fl=%d fr=%d cr=%d r=%d, (%d,%d,%d,%d,%d,%d),%d\r\n", \
            signal_offset_ex[LT_LEFT-8],signal_offset_ex[LT_CENTERLEFT-8],signal_offset_ex[LT_FRONTLEFT-8],signal_offset_ex[LT_FRONTRIGHT-8],signal_offset_ex[LT_CENTERRIGHT-8],signal_offset_ex[LT_RIGHT-8],\
            signal_offset[LT_LEFT-8],signal_offset[LT_CENTERLEFT-8],signal_offset[LT_FRONTLEFT-8],signal_offset[LT_FRONTRIGHT-8],signal_offset[LT_CENTERRIGHT-8],signal_offset[LT_RIGHT-8],\
            lt_same_count);
			#endif
			#if 1
			 printf("touch: l=%d cl=%d fl=%d fr=%d cr=%d r=%d, (%d,%d,%d,%d,%d,%d)\r\n", \
            signal_delta[LT_LEFT], signal_delta[LT_CENTERLEFT], signal_delta[LT_FRONTLEFT], \
            signal_delta[LT_FRONTRIGHT], signal_delta[LT_CENTERRIGHT], signal_delta[LT_RIGHT],\
            signal_offset[LT_LEFT-8],signal_offset[LT_CENTERLEFT-8],signal_offset[LT_FRONTLEFT-8],signal_offset[LT_FRONTRIGHT-8],signal_offset[LT_CENTERRIGHT-8],signal_offset[LT_RIGHT-8]);      
      #endif
     // printf("touch thod(%d,%d,%d,%d,%d,%d)\r\n", signal_offset[LT_LEFT-LT_CENTERRIGHT],signal_offset[LT_CENTERLEFT-LT_CENTERRIGHT],signal_offset[LT_FRONTLEFT-LT_CENTERRIGHT],signal_offset[LT_FRONTRIGHT-LT_CENTERRIGHT],signal_offset[LT_CENTERRIGHT-LT_CENTERRIGHT],signal_offset[LT_RIGHT-LT_CENTERRIGHT]);            
    }
#endif
}
s16 cliff_data_return(u8 cliffnum)
{
	robot_sensor_gather_start(1);
    if(cliffnum==1)  {
		return signal_delta[CLIFF_LEFT];
	}else if(cliffnum==2){
		return signal_delta[CLIFF_FRONTLEFT];
	}else if(cliffnum==3){
		return signal_delta[CLIFF_RIGHT];
	}
}
s16 LT_data_return(u8 LTnum)
{
	robot_sensor_gather_start(1);
    if(LTnum==1)  {
		return signal_delta[LT_LEFT];
	}else if(LTnum==2){
		return signal_delta[LT_CENTERLEFT];
	}else if(LTnum==3){
		return signal_delta[LT_FRONTLEFT];
	}else if(LTnum==4){
		return signal_delta[LT_CENTERRIGHT];
	}else if(LTnum==5){
		return signal_delta[LT_RIGHT];
	}
}

