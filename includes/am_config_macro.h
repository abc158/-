#ifndef __CONFIG_MACRO_H__
#define __CONFIG_MACRO_H__
///////////////////////cliff config///////////////////////

//#define USE_UART_WIFI
#ifdef USE_UART_WIFI
//#define USE_WIFI_DEMO_1
//#define USE_WIFI_DEMO_2
#endif
typedef enum
{  
  CLIFF_REAR_LEFT_THRESHOLD_ON         = -1, /*����*/
  CLIFF_REAR_LEFT_THRESHOLD_OFF        = -1, 
  CLIFF_REAR_RIGHT_THRESHOLD_ON        = -1, /*����*/
  CLIFF_REAR_RIGHT_THRESHOLD_OFF       = -1, 
  CLIFF_LEFT_THRESHOLD_ON              = 500, /*��*/
  CLIFF_LEFT_THRESHOLD_OFF             = 500, 
  CLIFF_RIGHT_THRESHOLD_ON             = 500, /*��*/
  CLIFF_RIGHT_THRESHOLD_OFF            = 500,
  CLIFF_FRONT_LEFT_THRESHOLD_ON        = 500, /*ǰ��*/
  CLIFF_FRONT_LEFT_THRESHOLD_OFF       = 500,
  CLIFF_FRONT_RIGHT_THRESHOLD_ON       = 500, /*ǰ��*/
  CLIFF_FRONT_RIGHT_THRESHOLD_OFF      = 500
}CLIFF_E;

/////////////////////ligth_touch config////////////////////////////

typedef enum
{
  LT_CENTERLEFT_THRESHOLD_ON    = 1500, /*�м���*/
  LT_CENTERLEFT_THRESHOLD_OFF   = 1500,
  LT_CENTERRIGHT_THRESHOLD_ON   = 1500, /*�м���*/
  LT_CENTERRIGHT_THRESHOLD_OFF  = 1500,
  LT_LEFT_THRESHOLD_ON          = 1500, /*��*/
  LT_LEFT_THRESHOLD_OFF         = 1500,
  LT_RIGHT_THRESHOLD_ON         = 1500, /*��*/
  LT_RIGHT_THRESHOLD_OFF        = 1500,
  //   LT_FRONT_LEFT_THRESHOLD_ON    = 95, /*ǰ��*/
  //  LT_FRONT_LEFT_THRESHOLD_OFF   = 95,
  LT_FRONT_LEFT_THRESHOLD_ON    = 300, /*ǰ��*/
  LT_FRONT_LEFT_THRESHOLD_OFF   = 300,
  LT_FRONT_RIGHT_THRESHOLD_ON   = 300, /*ǰ��*/
  LT_FRONT_RIGHT_THRESHOLD_OFF  = 300
}LT_E;

/*������*/
typedef enum
{
  OFF=0x0, 
  ON =0x1  
}EN_E;

/*Ӧ��״̬*/
typedef enum
{
  RESP_OK  =0x0, /*����ok*/
  RESP_ERR =0x1  /*����err*/
}RESPONSE_E;

/*Ӧ��״̬*/
typedef enum
{
  PASS  =0x0, /*���Գɹ�*/
  FAIL  =0x1  /*����ʧ��*/
}TEST_STATE_E;


/*ң��������ֵ*/
typedef enum
{
  IR_DOCK  = 0xC2,
  IR_SPOT  = 0xCA,
  IR_CLEAN = 0xC3,
  IR_POWER = 0xC1,
  IR_LEFT  = 0xC5,
  IR_RIGHT = 0xC6,
  IR_FORWARD=0xC4,
  IR_SCHEDULE=0xCC, 
  IR_CLOCK   =0xCF, //0xCB
  IR_DRIVERSTOP=0x8d,
  IR_GRID     = 0xC9, //0xC8
  IR_BACKWARD = 0xC7,
  IR_WALLFOLLOW = 0xCB,//0xC9,
  IR_DIRT  = 0xC8,//0x22, //����
  IR_CLEAN_DRIVE = 0x105,
  IR_ADJUST  = 0xC8,
  IR_SOUND  = 0xCD,
  IR_RANDOM = 0xFF,
}IR_E;


/*��������������ֵ*/
typedef enum
{
  DOCK_FORCE_FIELD = 0xb1,  //Բ����ֵ
  DOCK_BUOY_RED    = 0xb4,  //��
  DOCK_BUOY_GREEN  = 0xb8,  //�� 
  DOCK_BUOY_BOTH   = 0xbc       
}DOCK_E;

/*lt bump info*/
typedef enum
{
  LT_FRONT_LEFT = 0x1,
  LT_FRONT_RIGHT= 0x2,
  LT_CENTER_LEFT= 0x4,
  LT_CENTER_RIGHT=0x8,
  LT_SIDE_LEFT   =0x10,
  LT_SIDE_RIGHT  =0x20
}LT_BUMP_ACTIVE_DIR_E;

typedef enum
{
  UI_SET_ROBOT_BUMP_USING_LIGHT_TOUCH = 1,
  UI_SET_LT_ACC_MMPS            = 800,
  UI_SET_LT_BUMP_THROD          = 400 ,
  UI_SET_LT_BUMP_LEFT_RIGHT_THROD = 600,
  UI_SET_LT_MAX_LIGHT_VALUE     = 1000,  
  UI_SET_LT_BUMP_ACTIVE_DIR     = LT_FRONT_LEFT|LT_FRONT_RIGHT,
}LT_BUMP_E;

typedef enum
{
  BAT_TYPE               = 0,
  ADAPTER_OUTPUT_VOL     = 22,
  ADAPTER_OUTPUT_CURRENT = 800,
  PA_GAIN                = 55,
  ACTION                 = 0x5,
  BAT_CAP_MAX            =  3000,
  BAT_CAP_MIN            =  0,
  BAT_VOL_MAX            =  3628,  //�����16.8Vʱ��Ӧ��ADC
  BAT_VOL_MIN            =  2700,  
  BAT_VOLTAGE_MIN        =  2790,  /*��͵�ع�����ѹ13000mV*/
  BAT_VOLTAGE_MAX        =  3628,  /*����ع�����ѹ16800mV*/
  BAT_VOLTAGE_DEAD       =  2700,  /*��ֹ������ѹ12.5v*/
  BAT_VOLTAGE_CHARGE_MAX =  3628,  /*�������ѹ 16.8v*/
  BAT_TEMPERATURE_MAX    =  50,    /*���й���������¶�50C*/
  WHEEL_CRASH_CURRENT    =  1500,   /*�ж���ײ*/    
  WHEEL_TWINE_CURRENT    =  2000,  /*�жϲ�ס*/
  MAIN_BRUSH_TWINE_CURRENT= 1000,  /*�жϲ�ס*/
  SIDE_BRUSH_TWINE_CURRENT= 800,   /*�жϲ�ס*/
  VACUUM_STALL_CURRENT    = 500,	/*�ж���ն�ס*/
  MAIN_BRUSH_MAX_VOLTAGE  = 2000, /*��ˢ���������ѹ*/
  MAIN_BRUSH_MIN_VOLTAGE  = 2000,  /*��ˢ����С������ѹ*/
  SIDE_BRUSH_MAX_VOLTAGE  = 1500, /*��ˢ���������ѹ*/
  SIDE_BRUSH_MIN_VOLTAGE  = 1500,  /*��ˢ����С������ѹ*/
  VACUUM_NORMAL_VOLTAGE      = 2590, /*1700*/
  VACUUM_ENHANCE_VOLTAGE      = 3600,
  MAIN_BRUSH_REVERSE_VOLTAGE = 2100,
  SIDE_BRUSH_REVERSE_VOLTAGE = 1950

}POWER_E;

/*���ϼ������ѡ��*/
typedef enum
{
  CRASH_OFF = 0x0, /*�ر��ϰ����*/
  BUMP_ON   = 0x1, /*bump���*/
  LT_ON     = 0x2, /*LT���*/
  LT_BUMP_ON= 0x3  /*BUMP and lt ON*/
}CRASH_COND_E;

/*����ṹ�ϵ�һЩ��Ϣ*/
#define  WHEEL_DIA                  69.0f
#define  WHEEL_LEFT_RIGHT_DISTANCE  200.7f
#define  WHEEL_CODE_RATIO           554.8f
#define  PAD_WIDTH                  315.0f

typedef enum
{
  CRASH_COND                = LT_BUMP_ON,  /*���ϼ������*/
  P_VALUE                   = 180,          
  I_VALUE                   = -380,
  D_VALUE                   = -300,
  HARDWARE_VERSION          = 1  
}HARD_STRUCTURE_E;

/*��Ϊ�ٶ���Ϣ*/

typedef enum
{
    CLEAN_SPEED_MAX  = 300, /*��ɨʱ����ٶ�*/
    CLEAN_SPEED_LOW  = 220,
    CRASH_SPEED      = 140,  /*�����ϰ��ｵ���ٶ�*/
    DOCK_SPEED_NOMAL = 150, /*�����������ٶ�*/
    DOCK_SPEED_LOW   = 70,   /*�ҵ������󽵵͵��ٶ�*/
    WALLFOLLOW_SPEED = 180  /*�ر���ɨ���ٶ�*/
}SPEED_E;

#endif /*end of config*/
