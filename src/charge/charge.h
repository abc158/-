/************************************************************************
** file: charge.h
** 描述：充电状态定义
** 作者：lyy
** 版本：v1.0
** 版本历史
*************************************************************************/
#ifndef CHARGE_H
#define CHARGE_H

#include "am_type.h"

#define CHARGE_ENABLE 0
#define CHARGE_DISABLE 1

//#define NiMH      //镍氢电池，则打开该宏
#define TEMP_EXPRESSIONS	//使用公式法计算温度则打开该宏
typedef enum
{
  CHARGING_OFF = 0, 
  CHARGING_RECOVERY = 1,
  CHARGING_ON = 2,
  CHARGING_PURLING = 3,
  CHARGING_WAITING = 4,
  CHARGING_COMPLETE = 5
}CHARGING_STATE_E;


typedef enum
{
  CHARGING_NONE = 0, 
  CHARGING_NOBATTERY = 1,
  CHARGING_OVERCURRENT = 2,
  CHARGING_NOCURRENT = 3,
  CHARGING_OVERTEMP = 4
}CHARGING_STATE_ERROR_E;


//电压计算关系bat_voltage = Vadc/0.174  bat adc = bat_voltage*0.174*4096/3.3
// 电池标称电压3.7v  4节  满电压4.2*4=16.8v  恢复电压3*4=12v 低电3.3*4=13.2v 
//注意：电压adc的计算跟原理图中提供的参数有很大关系，有些电路参数是：0.174，有些是：0.18

#define BAT_CAP 	2200  //电池容量 单位：mA

#define VOL_PARM		0.18333	//根据电压计算adc的参数，从原理图中获得
#define ADC_CALC(vol)	(int)(vol*VOL_PARM*4096/3.3)  //adc的计算



//镍氢电池
#ifdef NiMH	
	#define NODE_NUMBER  				12	//镍氢电池节数

	//有关电压
	#define CHARGING_MAXVOL           ADC_CALC(1.4*NODE_NUMBER) 	//1.4*12=16.8V   	//充满电时的最大电压（开路电压）	
	#define CHARGE_VOLTAGE_RECOVERY   ADC_CALC(1*NODE_NUMBER) 		//12v				//小于该电压时需要先用小电流进行预充电
	#define CHARGING_OVER_PROTECTVOL  ADC_CALC(1.6*NODE_NUMBER) 	//1.6*12 = 19.2V 	//过充保护电压
	#define CHARGE_VOLTAGE_RESTART    CHARGING_MAXVOL-ADC_CALC(0.04*NODE_NUMBER)   		//充满电之后如果电压回落到该电压则重新充电

	//充满条件
	#define TEMP_VARIANT  				0.8f   					//温度差 单位：℃
	#define FALL_VOL					5*NODE_NUMBER			//电压差 单位：mv	每节电池回落5mv，12节电池就是60mv

	//热敏电阻参数，请参考电池规格说明书进行修改
	#define NET_PAR_B	3950	//B常量
	#define NET_PAR_T2	25		//T2温度，单位：℃，该温度一般取常温，也就是25℃
	#define NET_PAR_R	10		//T2温度下的阻值，单位：千欧

	
	//表格法求温度
	#define TEMP_NUMBER 30	
	typedef struct
	{
	  s16  temp; //温度  
	  u16  r;    //电阻
	}ntc_map_t;
	
//锂电池	
#else	
	#define NODE_NUMBER  				4	//锂电池节数
	
	//有关电压
	#define CHARGING_MAXVOL           ADC_CALC(4.2*NODE_NUMBER) 	//4.2*4=16.8V   	//充满电时的最大电压（开路电压）	
	#define CHARGE_VOLTAGE_RECOVERY   ADC_CALC(3*NODE_NUMBER) 		//3*4=12v			//小于该电压时需要先用小电流进行预充电
	#define CHARGING_OVER_PROTECTVOL  ADC_CALC(4.28*NODE_NUMBER) 	//4.28*4=17.12v		//过充保护电压
	#define CHARGE_VOLTAGE_RESTART    CHARGING_MAXVOL-30  			//充满电之后如果电压回落到该电压则重新充电
	
	//充满条件	
	#define CHARGING_CURRENT_STOP		0.01*BAT_CAP	//电流小于0.01C则充满
	
#endif


//充电电流
#define CHARGING_CURRENT_RECOVERY  	BAT_CAP*0.1		//0.1C
#define CHARGING_CURRENT_ON_MIN     200
#define CHARGING_CURRENT_ON_MAX     800
#define CHARGING_CURRENT_PURLING    BAT_CAP*0.05	//0.05C

	
//充电截止时间
#define CHARGE_MAX_DURATION_RECOVERY  100*60*60*((float)BAT_CAP/(float)(CHARGING_CURRENT_RECOVERY))   		//10 hours
#define CHARGE_MAX_DURATION_ON        100*60*60*(((float)BAT_CAP/(float)(CHARGING_CURRENT_ON_MAX))+1.0)  //3.5 hours
#define CHARGE_MAX_DURATION_PURLING   100*60*30   	//30 minutes


#define CHARGING_ON_PURLING_TIME      100*60*1         		//1 minutes     
#define CHARGE_RESTART_TIME           100*60*1          	//1 minutes
#define CHARGE_VOLTAGE_FULL_TIME      100*60*1          	//1 minutes
#define CHARGE_CURRENT_FULL_TIME      100*60*1          	//1 minutes
#define CHARGE_RESTART_TIME_FROM_FULL_CHARGE 100*60*60*4  	//4 hours

#define CHARGING_LOW_VOLTAGE      2851 //13.2V			//机器低电自动回座电压
#define CHARGING_CUTOFF_VOLTAGE   2634 //12.2v 			//机器自动关机电压
#define BATTERY_LOW_SLOWVECTORY   2916 //13.5v			//低电减速电压
	
extern void  charging_init(void);
extern void  process_charging(void);
extern void  quit_charging(void);
extern u16   charging_state_get(void);
U16 charging_jack(void);
U16 charging_dock(void);
s16 convert_to_voltage( s16 v_adc);
s16 convert_to_current( s16 c_adc);
S16 charge_current_get(void);
u8 get_charging_mode(void);
#endif