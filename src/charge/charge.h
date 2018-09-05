/************************************************************************
** file: charge.h
** ���������״̬����
** ���ߣ�lyy
** �汾��v1.0
** �汾��ʷ
*************************************************************************/
#ifndef CHARGE_H
#define CHARGE_H

#include "am_type.h"

#define CHARGE_ENABLE 0
#define CHARGE_DISABLE 1

//#define NiMH      //�����أ���򿪸ú�
#define TEMP_EXPRESSIONS	//ʹ�ù�ʽ�������¶���򿪸ú�
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


//��ѹ�����ϵbat_voltage = Vadc/0.174  bat adc = bat_voltage*0.174*4096/3.3
// ��ر�Ƶ�ѹ3.7v  4��  ����ѹ4.2*4=16.8v  �ָ���ѹ3*4=12v �͵�3.3*4=13.2v 
//ע�⣺��ѹadc�ļ����ԭ��ͼ���ṩ�Ĳ����кܴ��ϵ����Щ��·�����ǣ�0.174����Щ�ǣ�0.18

#define BAT_CAP 	2200  //������� ��λ��mA

#define VOL_PARM		0.18333	//���ݵ�ѹ����adc�Ĳ�������ԭ��ͼ�л��
#define ADC_CALC(vol)	(int)(vol*VOL_PARM*4096/3.3)  //adc�ļ���



//������
#ifdef NiMH	
	#define NODE_NUMBER  				12	//�����ؽ���

	//�йص�ѹ
	#define CHARGING_MAXVOL           ADC_CALC(1.4*NODE_NUMBER) 	//1.4*12=16.8V   	//������ʱ������ѹ����·��ѹ��	
	#define CHARGE_VOLTAGE_RECOVERY   ADC_CALC(1*NODE_NUMBER) 		//12v				//С�ڸõ�ѹʱ��Ҫ����С��������Ԥ���
	#define CHARGING_OVER_PROTECTVOL  ADC_CALC(1.6*NODE_NUMBER) 	//1.6*12 = 19.2V 	//���䱣����ѹ
	#define CHARGE_VOLTAGE_RESTART    CHARGING_MAXVOL-ADC_CALC(0.04*NODE_NUMBER)   		//������֮�������ѹ���䵽�õ�ѹ�����³��

	//��������
	#define TEMP_VARIANT  				0.8f   					//�¶Ȳ� ��λ����
	#define FALL_VOL					5*NODE_NUMBER			//��ѹ�� ��λ��mv	ÿ�ڵ�ػ���5mv��12�ڵ�ؾ���60mv

	//���������������ο���ع��˵��������޸�
	#define NET_PAR_B	3950	//B����
	#define NET_PAR_T2	25		//T2�¶ȣ���λ���棬���¶�һ��ȡ���£�Ҳ����25��
	#define NET_PAR_R	10		//T2�¶��µ���ֵ����λ��ǧŷ

	
	//������¶�
	#define TEMP_NUMBER 30	
	typedef struct
	{
	  s16  temp; //�¶�  
	  u16  r;    //����
	}ntc_map_t;
	
//﮵��	
#else	
	#define NODE_NUMBER  				4	//﮵�ؽ���
	
	//�йص�ѹ
	#define CHARGING_MAXVOL           ADC_CALC(4.2*NODE_NUMBER) 	//4.2*4=16.8V   	//������ʱ������ѹ����·��ѹ��	
	#define CHARGE_VOLTAGE_RECOVERY   ADC_CALC(3*NODE_NUMBER) 		//3*4=12v			//С�ڸõ�ѹʱ��Ҫ����С��������Ԥ���
	#define CHARGING_OVER_PROTECTVOL  ADC_CALC(4.28*NODE_NUMBER) 	//4.28*4=17.12v		//���䱣����ѹ
	#define CHARGE_VOLTAGE_RESTART    CHARGING_MAXVOL-30  			//������֮�������ѹ���䵽�õ�ѹ�����³��
	
	//��������	
	#define CHARGING_CURRENT_STOP		0.01*BAT_CAP	//����С��0.01C�����
	
#endif


//������
#define CHARGING_CURRENT_RECOVERY  	BAT_CAP*0.1		//0.1C
#define CHARGING_CURRENT_ON_MIN     200
#define CHARGING_CURRENT_ON_MAX     800
#define CHARGING_CURRENT_PURLING    BAT_CAP*0.05	//0.05C

	
//����ֹʱ��
#define CHARGE_MAX_DURATION_RECOVERY  100*60*60*((float)BAT_CAP/(float)(CHARGING_CURRENT_RECOVERY))   		//10 hours
#define CHARGE_MAX_DURATION_ON        100*60*60*(((float)BAT_CAP/(float)(CHARGING_CURRENT_ON_MAX))+1.0)  //3.5 hours
#define CHARGE_MAX_DURATION_PURLING   100*60*30   	//30 minutes


#define CHARGING_ON_PURLING_TIME      100*60*1         		//1 minutes     
#define CHARGE_RESTART_TIME           100*60*1          	//1 minutes
#define CHARGE_VOLTAGE_FULL_TIME      100*60*1          	//1 minutes
#define CHARGE_CURRENT_FULL_TIME      100*60*1          	//1 minutes
#define CHARGE_RESTART_TIME_FROM_FULL_CHARGE 100*60*60*4  	//4 hours

#define CHARGING_LOW_VOLTAGE      2851 //13.2V			//�����͵��Զ�������ѹ
#define CHARGING_CUTOFF_VOLTAGE   2634 //12.2v 			//�����Զ��ػ���ѹ
#define BATTERY_LOW_SLOWVECTORY   2916 //13.5v			//�͵���ٵ�ѹ
	
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