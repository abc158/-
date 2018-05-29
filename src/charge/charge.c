//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/
#include "charge.h"
#include "syscall_api.h"
#include "util/current.h"
#include "am_date_base.h"
#include "ui-song-player.h"
#include "am_config_macro.h"
#include "math.h"

#define OPEN_DEBUG_PRINT_CHARGING

#ifdef OPEN_DEBUG_PRINT_CHARGING
  #define CHARGING_DEBUG_PRINT(...)    printf(__VA_ARGS__);
#else
  #define CHARGING_DEBUG_PRINT(...)
#endif
#define MAX_VOLT      0xffff

u16  g_static_voltage = MAX_VOLT;
u8 g_time_out = 0;
u32 current_overflow_cnt = 0;
u32 charging_time = 0;
u32 discharge_time = 0;
u32 trickle_charging_time = 0;
u32 charge_restart_cnt = 0;
u16 charge_neg_cnt =0;
u16 charge_vol_over_cnt =0;
u16 charge_over_temp =0;
u16 max_voltage;
float last_temp =0;
u16 nimh_charge_stop_cnt;
u8  comlete_charged =0;
u8  charging_state_last =6;

u16 before_charging_volt =0;
CHARGING_STATE_E charging_state = CHARGING_OFF;
extern const IO_PIN_CFG io_table[];
extern u8 charging_detect(void);
void reset_charge_judge_condition(void);

#ifdef NiMH
#ifndef TEMP_EXPRESSIONS 
//����¶Ⱥ�����������ֵ�Ķ�Ӧ��ϵ��  �¶ȵ�λ�����϶�     ��ֵ��λ��ŷ 
ntc_map_t temp_r_map[TEMP_NUMBER]=\
{
  	20,12535,
	21,11974,
	22,11441,
	23,10936,
	24,10456,
	25,10000,
	26,9567,
	27,9155,
	28,8764,
	29,8391,
	30,8037,
	31,7700,
	32,7379,
	33,7074,
	34,6783,
	35,6506,
	36,6241,
	37,5989,
	38,5749,
	39,5520,
	40,5301,
	41,5093,
	42,4894,
	43,4703,
	44,4522,
	45,4348,
	46,4182,
	47,4024,
	48,3872,
	49,3727
};
#endif

/****************************************************************
*Function   :  calc_battery_temperature
*Author     :  Ljh    
*Date       :  2017.11.26
*Description:  �������¶�
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ����¶� ����λ���ȣ�
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    Ljh            17.11.26      v1.0         build this function
******************************************************************/
#ifdef TEMP_EXPRESSIONS 	//�����ṩ�˹�ʽ���Ͳ�������ַ��������¶ȵļ���
float calc_battery_temperature(void)
{
  	static float t;
	volatile float r = battery_temperature_get();
	t=(1/((log(r/NET_PAR_R)/NET_PAR_B)+1/(273.15+NET_PAR_T2)))-273.15;
	return t;	
}
#else
float calc_battery_temperature(void)
{
 	u8 i;
	float  temp;
	volatile u32 r = (int)(battery_temperature_get()*1000);	
	//printf("  r=%d\r\n",r);
  	for(i =0;i < TEMP_NUMBER;i++)
	{
		if(r > temp_r_map[i].r)
		{	
			if(i ==0)
				temp =(float)temp_r_map[i].temp;	
			else
				temp =(float)temp_r_map[i-1].temp;	
			return temp;
		}	
	}
	temp =(float)temp_r_map[i].temp;	
	return temp;	
}
#endif
#endif
/****************************************************************
*Function   :  nimh_charge_is_complete
*Author     :  Ljh
*Date       :  2017.11.24
*Description:  ��ȡ����¶�
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ����¶� ��λ����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24      v1.0         build this function
******************************************************************/
float get_battery_temp(void) 
{    
	return last_temp;   	
}

/****************************************************************
*Function   :  check_charge_restart
*Author     :  ljh
*Date       :  2017.11.24
*Description:  ���õ�ǰ������ (��λ��mA)
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24       v1.0         build this function
******************************************************************/
void set_charging_current(u16 val)
{
  	static u16 last_val;
	if(val != last_val)
	{
		if(val >0)
		{
			set_charge_current(val);
			set_charging_enable(1);//�����õ����ٴ򿪳��ʹ��
			last_val =val;	
		}
		else
		{	
			set_charge_current(0);				
			set_charging_enable(0);//close charging
			last_val = val;
		}		
	}
}

/****************************************************************
*Function   :  voltage_to_adc
*Author     :  Ljh    
*Date       :  2017.11.26
*Description:  �ѵ�ѹ����λmv��ת���ɵ�ѹadc
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ����¶� ����λ���ȣ�
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    Ljh            17.11.26      v1.0         build this function
******************************************************************/
s16 voltage_to_adc( s16 mv) 
{
  s16 v_adc;
  v_adc =(int)((mv<<12)*VOL_PARM/(3.3*1000));//0.18 ---> 18032; 0.174 --> 18975 ///3.3/0.174*1000=18975
  return (s16)v_adc;
}
/****************************************************************
*Function   :  convert_to_voltage
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ѵ�ѹadcֵת�ɵ�ѹ����λmv��
*CallBy     :  
*Input      :  ����
*              v_adc �� ��ѹ��adc
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
s16 convert_to_voltage( s16 v_adc) 
{
  s32 mv= (int)((3.3*1000*v_adc)/VOL_PARM);//0.18 ---> 18032; 0.174 --> 18975 ///3.3/0.174*1000=18975
  mv = (mv>>12);
  return (s16)mv;
}

/****************************************************************
*Function   :  convert_to_current
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ѵ���adcֵת�ɵ�ѹ����λmA��
*CallBy     :  
*Input      :  ����
*              v_adc �� ������adc
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
s16 convert_to_current( s16 c_adc) 
{
  s16 ret =  (s16)(((0.95 - c_adc*3.3/4096.0f)/0.41)*1000);
  return ret;
}

/****************************************************************
*Function   :  charging_print
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ӡ��������Ϣ����ѹ�����������״̬��
*CallBy     :  һ���ڳ������
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  �����ڱ���ʵ��ͬ�����ܺ���
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void charging_print(void)
{
  	static u32 one_second=0;

	if(charging_state != charging_state_last )//ÿ��״̬����ת�䶼��ӡ����
	{
		printf("TURN:%d\r\n",charging_state);
		charging_state_last =charging_state;
	} 
	if(get_total_seconds()>(one_second+10))
	{
		one_second = get_total_seconds();
		float current =charge_current_get();		
		printf("charging state:%d",get_ui_manager()->charging_state());//charging_state
		printf(" v:%d c:%d mA ",battery_voltage_average(),(int)current); 	
#ifdef NiMH		
		printf(" t:%d",(int)(calc_battery_temperature()*100)); 	
		printf(" r=%d",(int)(battery_temperature_get()*1000));
#endif		
		printf("\r\n");
	}
}

/****************************************************************
*Function   :  charging_init
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ʼ��������
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void charging_init(void)
{ 
	g_time_out = 0;
	current_overflow_cnt = 0;
	charging_time = 0;
	discharge_time = 0;
	trickle_charging_time = 0;
	charging_state = CHARGING_OFF;
	charge_restart_cnt = 0;
	reset_charge_judge_condition();

#ifdef NiMH
  set_charge_voltage(CHARGING_OVER_PROTECTVOL+100);//�����ز���Ҫ��ѹ��磬���԰Ѹõ�ѹ��߲���������ѹ��磬��ѹ�׶���SDK�е��������
#else
  set_charge_voltage(CHARGING_MAXVOL);
#endif 
  set_adaptor_output_voltage(ADAPTER_OUTPUT_VOL);
  set_charging_current(0);	
}

/****************************************************************
*Function   :  charging_state_get
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ��ȡ���״̬
*CallBy     :  �κεط���Ŀǰ����ӿ��Ǳ���ʵ�֣���Ϊע���SDKʹ��
*Input      :  ��
*Output     :  ��
*Return     :  ���� charging_state�� ���״̬
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
u16 charging_state_get(void)
{
  return charging_state;
}

/****************************************************************
*Function   :  charge_current_get
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ��ǰʵ�ʵĳ����� (��λ��mA)
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ����
*Output     :  ��
*Return     :  ���� ʵ�ʵĳ�����ֵ (mA)
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
S16 charge_current_get(void)
{
    return convert_to_current(get_adc_chan_val( ADC_CHAN_BATTERY_CURRENT ));   
}

/****************************************************************
*Function   :  before_charging_handle
*Author     :  ljh
*Date       :  2017.11.24
*Description:  ���֮ǰ����Ҫ���Ĵ������Է������������
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24       v1.0         build this function
******************************************************************/
u8 begin_charge_report = 0;
static int play_song_cnt = 0;
void play_charging_song(void)
{
  if(!begin_charge_report)
  {
    play_song_cnt++;
    if(play_song_cnt > 200)//2s������
    {
      if( sys_runing_mode_get()==ROBOT_STATE_CHARGING) 
      {
        songplayer_play_id(SONG_ID_CHARGING_START, 0);
		printf("begin_charging\r\n");
        begin_charge_report = 1;
      }
    }       
  }
}


/****************************************************************
*Function   :  before_charging_handle
*Author     :  ljh
*Date       :  2017.11.24
*Description:  ���֮ǰ����Ҫ���Ĵ������Է������������
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24       v1.0         build this function
******************************************************************/
u8 before_charging_handle(void)
{ 
  if(before_charging_volt == 0)//�ú���ֻ�ڸս�����ʱ��ִ��һ��,��һЩ���⴦��
  {
	before_charging_volt = battery_voltage_average();
	printf("before volt = %d \r\n",before_charging_volt);	
	//���֮ǰ�ǳ����������Ǳ����������Ż��������ʱ��Ӧ�����ѳ�����״̬
	if( (before_charging_volt > (g_static_voltage-15)) && \
		 (before_charging_volt >(CHARGING_MAXVOL-50)) )
	{
		g_static_voltage = MAX_VOLT;
		CHARGING_DEBUG_PRINT(" just now charge complete\r\n");
		return 1;
	}
  }
  return 0;
}
/****************************************************************
*Function   :  nimh_charge_is_complete
*Author     :  Ljh
*Date       :  2017.11.24
*Description:  ���ݵ�ѹ���ж��������Ƿ������
*CallBy     :  �κεط����жϳ���
*Input      :  �����ѹ��
*Output     :  ��
*Return     :  ���� 1�� ������   0��δ����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24      v1.0         build this function
******************************************************************/
static S8 nimh_charge_is_complete(s16 mv) 
{   
	static u16 currently_voltage;
	static u8 cnt;
	
	if(max_voltage ==0)//�ղ����������Զ�ȡ������ѹ���д���
	{
		if(charging_time > 100*10)//10s֮��ſ�ʼ���㣬����ղ�����������ʱ���ȡ������������ӿ��ѹ	  	
		{
			max_voltage = battery_voltage_average();
			nimh_charge_stop_cnt=0;
		}
		return 0;
	}
	if(charging_time == 0)//����֮��������������Ҫ�����ֵ���¸�ֵ
	{
		max_voltage = battery_voltage_average();
		nimh_charge_stop_cnt=0;
	}		
	
	currently_voltage = battery_voltage_average();
	//printf("v=%d\r\n",currently_voltage);
	if(currently_voltage > max_voltage)
	{
		cnt++;
		if(cnt > 5)	//����
		{
			max_voltage = currently_voltage;
			cnt =0;
			//CHARGING_DEBUG_PRINT("charging_max_vol=%d\r\n",max_voltage);
		}			
	}
	else
	{
		if(cnt >0)
			cnt--;
	}		
	
	if((currently_voltage < max_voltage)&&(check_battery_voltage(CHARGING_MAXVOL))) //��ѹ������
	{
	  	if(nimh_charge_stop_cnt <65535)//�����������65535֮���ͷ��������ͷ����֮����2min�����������Ǹ��ж��С���Ϊ���ָ�������һ���ܿ�Ϳ����жϵ���ѹ�½���50mv
			nimh_charge_stop_cnt++;
	}
	else
	{
//	  	if(nimh_charge_stop_cnt >0)//�������0֮���65535��ʼ���¼�
//			nimh_charge_stop_cnt--;
		nimh_charge_stop_cnt =0;
	} 
	if(nimh_charge_stop_cnt > 100*60*2)//2min
	{	  	
		if((max_voltage-currently_voltage) > voltage_to_adc(mv)) //ÿ�ڵ�ص�ѹ���½�5mv������ȡ60mv��Ӧ��adcΪ13
		{
			nimh_charge_stop_cnt =0;			
			CHARGING_DEBUG_PRINT("max:%d,current_vol:%d,dif:%d\r\n",max_voltage,currently_voltage,(max_voltage-currently_voltage));	
			return 1;
		}			
	}	   
	return 0;   	
}
/****************************************************************
*Function   :  check_battery_temperature_different
*Author     :  Ljh    
*Date       :  2017.8.18
*Description:  �涨ʱ���ڵ��¶Ȳ��Ƿ��������Ĳ���
*CallBy     :  �κεط����ж������ĳ���
*Input      :  �¶Ȳ�ֵ����λ����
*Output     :  ��
*Return     :  1�����ڲο��¶�   0�������ڲο��¶�
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    Ljh            17.8.18      v1.0         build this function
******************************************************************/
u8 check_battery_temperature_different(float t)
{
    static u32 one_second=0;
	static float temp_t=0;
	u8 flag =0;
	if(charging_time < 5)//ÿ�β�����������Ҫ�Ը��м�������¸�ֵ.����������ʱ���¶ȱȽϸߣ�Ȼ���Ż��ο�������һ��ʱ��֮�����²��ϣ��¶Ƚ������ˣ���ֱ���жϵ�һ���Ƚϴ���²�γ����С�
	{
		temp_t = get_battery_temp();
		one_second = get_total_seconds();
		flag =0;
	}
	
    if(get_total_seconds()>(one_second+60))//60s
    {
        one_second = get_total_seconds();
				
		if(((get_battery_temp()-temp_t) > t) && ((get_battery_temp()-temp_t) < 3))//�²����t,�Ҳ�����3��
		{		
		  	CHARGING_DEBUG_PRINT("temp dif over:%d\r\n",(int)((get_battery_temp()-temp_t)*100));
			flag =1;		
		}
		else
		{
			flag =0;
		}
		//CHARGING_DEBUG_PRINT("--temp dif:%d\r\n",(int)((get_battery_temp()-temp_t)*100));
		temp_t = get_battery_temp();
    }
	return flag;
}
/****************************************************************
*Function   :  over_charge_protect
*Author     :  Ljh
*Date       :  2017.8.16
*Description:  ���䱣������
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.8.16      v1.0         build this function
******************************************************************/
static u8 over_charge_protect(void)
{
#ifdef NiMH
  	if(charging_state_get() == CHARGING_ON)
	{
		if(get_battery_temp()>46.0f)//���ٳ��ʱ�¶�������45��,���ǵ����������ø�һ��
		{
			charge_over_temp++;
		}
		else
		{
			if(charge_over_temp>0)
			  charge_over_temp--;	
		}		
	}
	else
	{
		if(get_battery_temp()>41.0f)//0.1C���ʱ�¶�������40��
		{
			charge_over_temp++;
		}
		else
		{
			if(charge_over_temp>0)
			  charge_over_temp--;	
		}			
	}
   
  	if(charge_over_temp > 100*30)//�¶ȴ��������¶ȳ���30s
	{	
	  	charge_over_temp =0;
	  	CHARGING_DEBUG_PRINT("over charge protect,temp:%d\r\n",(int)(get_battery_temp()*100));
		return 1;	 	
	} 
#endif 
	
	if(charge_current_get() < 0)
	{
		charge_neg_cnt++;
	}
	else
	{
		if(charge_neg_cnt>0)
	 		charge_neg_cnt--;
	}		
		
	if(charge_neg_cnt > 100*60)	 //������Ϊ������ʾ�ŵ磬����60s�����
	{
	    charge_neg_cnt  = 0;
		CHARGING_DEBUG_PRINT("over charge protect,neg current:%d\r\n",charge_current_get());
		return 1;	 
	}
//////////////////////////////////////////////////////////////////	
	if(check_battery_voltage(CHARGING_OVER_PROTECTVOL))
	{
	  	charge_vol_over_cnt++;
	}
	else
	{
		if(charge_vol_over_cnt>0)
		  charge_vol_over_cnt--;	
	}	
	if(charge_vol_over_cnt > 100*30)//��ѹ���ڱ�����ѹ30s	  
	{
	    charge_vol_over_cnt  = 0;
		CHARGING_DEBUG_PRINT("over charge protect,vol:%d\r\n",battery_voltage_average());
		return 1; 
	}		
	
	return 0;
}
/****************************************************************
*Function   :  check_charge_restart
*Author     :  ljh
*Date       :  2017.11.24
*Description:  �ж��Ƿ���Ҫ���³��
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24       v1.0         build this function
******************************************************************/
u8 check_charge_restart(void)
{
	if (g_time_out == 0)
	{
		if (!check_battery_voltage(CHARGE_VOLTAGE_RESTART))
		{
            charge_restart_cnt++;
		}
		if (charge_restart_cnt >= CHARGE_RESTART_TIME)
		{       
            charge_restart_cnt =0;
			reset_charge_judge_condition();
            CHARGING_DEBUG_PRINT("voltage under restart_vol:%d-%d\r\n",CHARGE_VOLTAGE_RESTART,battery_voltage_average());
            return 1;
		}
    }
  //�����������ϲ�����״̬�£����ĵ����������ĵ磬��ز��������������ģ����Կ��Բ����������  
//	if (discharge_time >= CHARGE_RESTART_TIME_FROM_FULL_CHARGE)//����֮��ÿ��4Сʱ�������һ�ε�
//	{
//	  	reset_charge_judge_condition();
//        CHARGING_DEBUG_PRINT("discharge_time over!\r\n");
//	 	return 1;
//	}
  
    return 0;
}
/****************************************************************
*Function   :  charge_full_detect
*Author     :  ljh
*Date       :  2017.11.24
*Description:  �жϵ���Ƿ����
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24       v1.0         build this function
******************************************************************/
u8 charge_full_detect(void)
{ 	 	
#ifdef 	NiMH
  	last_temp = calc_battery_temperature();
#endif
	
  	if(over_charge_protect())	//���䱣��
	{		
		return 1;
	}   	
	
#ifdef NiMH		
  if(nimh_charge_is_complete(FALL_VOL) || check_battery_temperature_different(TEMP_VARIANT))//�����������
  {
  	return 1;
  }	  
#else    
  if (charge_current_get() < CHARGING_CURRENT_STOP)
  {
    current_overflow_cnt++;
  }
  else
  {
  	current_overflow_cnt = 0;
  }  
  if (current_overflow_cnt >= CHARGE_CURRENT_FULL_TIME)//﮵�����������������С�ڽ�ֹ��������1����
  {
  	current_overflow_cnt = 0;
	CHARGING_DEBUG_PRINT("current under stop_current :%d\r\n",charge_current_get());
    return 1;
  }
#endif  
  
  //�������������״̬�ĳ�ʱ����
  if (charging_state == CHARGING_RECOVERY)
  {
    if (charging_time > CHARGE_MAX_DURATION_RECOVERY)
    {
    	g_time_out = 1;
		CHARGING_DEBUG_PRINT("time out:recovery\r\n");
        return 1;
    }
  }
  else if (charging_state == CHARGING_ON)
  {
    if (charging_time > CHARGE_MAX_DURATION_ON)
    {
    	g_time_out = 1;
		CHARGING_DEBUG_PRINT("time out:on\r\n");
        return 1;
    }
  }
  else if (charging_state == CHARGING_PURLING)
  {
    if (charging_time > CHARGE_MAX_DURATION_PURLING)
    {
    	g_time_out = 1;
		CHARGING_DEBUG_PRINT("time out:purling\r\n");
        return 1;
    }
  }

  return 0;
}

/****************************************************************
*Function   :  reset_charge_judge_condition
*Author     :  ljh
*Date       :  2017.11.24
*Description:  �ָ��жϳ������жϹ����״̬
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24       v1.0         build this function
******************************************************************/
void reset_charge_judge_condition(void)
{
  	charge_over_temp =0;
	charge_neg_cnt =0;
	charge_vol_over_cnt =0;
	
	current_overflow_cnt =0;
}
/****************************************************************
*Function   :  charging_state_conversion
*Author     :  ljh
*Date       :  2017.11.24
*Description:  ���״̬����ת������
*CallBy     :  �κεط����жϳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    ljh            17.11.24       v1.0         build this function
******************************************************************/
void charging_state_conversion(void)
{
	switch(charging_state)
    {
    case CHARGING_OFF:   
	  v_ref_init();
	  
	  if(before_charging_handle()) 
	  {
	  	charging_state = CHARGING_PURLING; 	    	
	  }
	  else
	  {
		  if (!(check_battery_voltage(CHARGE_VOLTAGE_RECOVERY)))
		  {
			charging_state = CHARGING_RECOVERY;     
		  }
		  else
		  {
			charging_state = CHARGING_ON;   
		  }	  
	  }
      break;
      
    case CHARGING_RECOVERY:
      set_charging_current(CHARGING_CURRENT_RECOVERY);
      if(check_battery_voltage(CHARGE_VOLTAGE_RECOVERY))
      {
		charging_time = 0;
        charging_state = CHARGING_ON;
      }
      if (charge_full_detect() == 1)
      {
        charging_state = CHARGING_COMPLETE;
      }     			
      break;
      
    case CHARGING_ON:   	
      if(trickle_charging_time < CHARGING_ON_PURLING_TIME)
      {
      	  trickle_charging_time++;
          set_charging_current(CHARGING_CURRENT_ON_MIN);
      }
      else
      {
          set_charging_current(CHARGING_CURRENT_ON_MAX); 
      }      
      if (charge_full_detect() == 1)
      {
        charging_state = CHARGING_COMPLETE;
      }
      break;
    
    case CHARGING_PURLING:     
      set_charging_current(CHARGING_CURRENT_PURLING);
	  comlete_charged =1;
      if (charge_full_detect() == 1)
      {
        charging_state = CHARGING_COMPLETE;
      }
      break;
            
    case CHARGING_COMPLETE:
      set_charging_current(0);
	  comlete_charged =1;
      if (check_charge_restart() == 1)
      {
		v_ref_init();
        charging_state = CHARGING_PURLING;
      }
      break;
    } 
	
	if (charging_state != CHARGING_COMPLETE)
    {    	
      charging_time++;
      discharge_time = 0;
    }
    else
    {
    	discharge_time++;
    	charging_time = 0;
    }
}

/****************************************************************
*Function   :  process_charging
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����������
*CallBy     :  Ŀǰ�Ǹ�SDK����
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void process_charging(void)
{ 	
	 //�����̿���
	charging_state_conversion();
    
    charging_print();
   	play_charging_song();
}


/****************************************************************
*Function   :  process_charging
*Author     :  lyy
*Date       :  2017.4.20
*Description:  �˳�����������
*CallBy     :  Ŀǰ�Ǹ�SDK����
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
void quit_charging(void)
{
    begin_charge_report  = 0;	
    before_charging_volt = 0;
    play_song_cnt = 0;
	max_voltage = 0;
	nimh_charge_stop_cnt =0;
	printf("state:%d\r\n",discharge_time);
	if(comlete_charged)
	{
		g_static_voltage = battery_voltage_average();
		comlete_charged =0;		
		printf("after volt:%d\r\n",g_static_voltage);
	}
	else
	{
		g_static_voltage = MAX_VOLT;
	}
    printf("quit charging volt:%d\r\n",battery_voltage_average());

}

/****************************************************************
*Function   :  charging_jack
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����Ƿ����������
*CallBy     :  �κεط�
*Input      :  ��
*Output     :  ��
*Return     :  1����������� 0����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
U16 charging_jack(void)
{
  	u8 dec_cnt = 0;
	u8 i = 0;
	
	for (i=0; i<10; i++)
	{
        if(gpio_get_value(IO_CHARGER_HIGH) == 0)
        {
            dec_cnt++;
        }
	}
	if (dec_cnt >= 6)
	{
        return 1;
	}
	else
	{
		return 0;
	}
  //return (gpio_get_value(IO_CHARGER_HIGH)==0);  
}

/****************************************************************
*Function   :  charging_dock
*Author     :  lyy
*Date       :  2017.4.20
*Description:  ����Ƿ��ڳ������
*CallBy     :  �κεط�
*Input      :  ��
*Output     :  ��
*Return     :  1����������� 0����
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
U16 charging_dock(void)
{
    u8 dec_cnt = 0;
	u8 i = 0;
	
	for (i=0; i<10; i++)
	{
        if(gpio_get_value(IO_CHARGER_LOW) == 0)
        {
            dec_cnt++;
        }
	}
	if (dec_cnt >= 6)
	{
        return 1;
	}
	else
	{
		return 0;
	}
  //return (gpio_get_value(IO_CHARGER_LOW)==0);
}


