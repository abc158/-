#ifndef _COMMAND_H__
#define _COMMAND_H__

/*命令状态*/
typedef enum
{
    CMD_STOP  =  0x0,/*停止命令*/
    CMD_RUN   =  0x1 /*执行命令*/
}CMD_STATE_E;

/*命令类型*/
typedef enum
{
    CONTROL_CMD  =  0x0,  /*控制命令*/
    CONFIG_CMD   =  0x1,  /*配置命令*/
    STATE_CMD    =  0x2,  /*状态命令*/	
    TEST_CMD     =  0x3,  /*测试命令*/
    DATA_CMD     =  0x4,  /*数据命令*/
    MP_TEST      =  0x5
}CMD_TYPE_E;

///////////////以下命令类型/////////////////
/*是以最高byte为0xf开头的32位,注意这个0xf有特殊的意义,所以不能更改,每一条命令必须是0xfxxxxxxx格式*/
/*控制命令类型*/
typedef enum
{
    CMD_DOCK               = 0xf0000001,   /*找充电座命令*/
    CMD_CLEAN              = 0xf0000002,   /*开始清扫、停止清扫命令*/
    CMD_SPOT               = 0xf0000004,   /*局部螺旋清扫命令*/
    CMD_SCHDULE            = 0xf0000008,   /*设置预约的命令*/ 
    CMD_CLOCK              = 0xf0000010,   /*设置系统时间命令*/
    CMD_POWER              = 0xf0000020,   /*进入休眠的命令*/   
    CMD_LEFT               = 0xf0000040,   /*向左转*/
    CMD_RIGHT              = 0xf0000080,   /*向右转*/
    CMD_FORWARD            = 0xf0000100,   /*向前走*/
    CMD_BACK               = 0xf0000200,   /*向后退*/
    CMD_SONG_ID            = 0xf0000400,   /*播放主板语音IC里面的语音*/
    CMD_WALL_FOLLOW        = 0xf0000800,   /*延边清扫*/
    CMD_RANDOM             = 0xf0001000,   /*随机清扫*/
    CMD_DIRT               = 0xf0002000,   /*重点清扫*/
    CMD_SET_GPIO           = 0xf0004000,   /*还没实现*/
    CMD_SHUTOFF            = 0xf0008000,   /*关机命令*/ 
    CMD_GRID1              = 0xf0008001,   /*保留*/
    CMD_GRID2              = 0xf0008002,   /*局部弓字形*/
    CMD_GRID3              = 0xf0008004,   /*保留*/
    CMD_OPEN_PA            = 0xf0008008,   /*开关PA*/
    CMD_SOUND_OUT          = 0xf0008010,   /*是否允许主板播放语音*/
    CMD_UV_ENABLE          = 0xf0008020,   /*开关UV灯*/
    CMD_EXIT               = 0xf0040000,//伪命令，内部用来控制方向键的停止
    CMD_MODE               = 0xf0040001, 
    CMD_SOS                = 0xf0040002,
    CMD_TESTDATA_DISPLAY   = 0xf0040003,//伪命令，内部用来控制方向键的停止
    CMD_TEST               = 0xf0040004,//伪命令，内部判断
    CMD_DISABLE_SENSOR     = 0xf0040005,
    
    CMD_ADJUST_MOTOR       = 0xf0040007,
    CMD_WIFI               = 0xf0040008,
    CMD_REST_WIFI          = 0xf0040009,
    CMD_POWER_ON           = 0xf004000a,
    CMD_POWER_DOWN         = 0xf004000b,
    CMD_MOP_CLEAN          = 0xf004000c,    /*拖地模式Clean*/
    CMD_GET_MAP            = 0xf004000d,    /*获取地图信息*/
    
    CMD_LONG_TEST          = 0xf0080001,    /*老化测试命令*/
    CMD_PAUSE              = 0xf0080002,
    CMD_TIMING             = 0xf0080003,    /*定时器功能*/
    CMD_FULL_GO            = 0xf0080004,    /*full go*/
    CMD_SET_ERR_NUM        = 0xf0008040,    /*设置出错ID*/
    CMD_SPEED_FAST         = 0xf004000e,    /*伪命令,快速*/
    CMD_SPEED_LOW          = 0xf004000f,    /*伪命令,慢速*/
    CMD_REMOTE_STOP        = 0xf0040010,    /**/
    CMD_MAX                = 0xf8000000
}CONTROL_CMD_E;


typedef enum
{
    CMD_TEST_WAIT      = 0x00,
    CMD_BUMP_L         = 0x01, /*测试主板左侧碰撞检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_BUMP_R         = 0x02, /*测试主板右侧碰撞检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_BUMP_B         = 0x03, /*测试主板左、右两侧同时碰撞的检测是否正常。主板回复的数据，0：失败，1：成功*/   
    CMD_CLIFF_L        = 0x04, /*测试主板左侧悬崖检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_CLIFF_R        = 0x05, /*测试主板右侧悬崖检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_CLIFF_FL       = 0x06, /*测试主板前左侧悬崖检测是否正常。主板回复的数据，0：失败，1：成 功*/
    CMD_CLIFF_FR       = 0x07, /*测试主板前右侧悬崖检测是否正常。主板回复的数据，0：失败，1：成 功*/
    CMD_CLIFF_RL       = 0x08, /*测试主板后左侧悬崖检测是否正常。主板回复的数据，0：失败，1：成 功*/
    CMD_CLIFF_RR       = 0x09, /*测试主板后右侧悬崖检测是否正常。主板回复的数据，0：失败，1：成 功*/                   
    CMD_LT_L           = 0x0a, /*测试主板左侧靠墙检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_LT_R           = 0x0b, /*测试主板右侧靠墙检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_LT_CL          = 0x0c, /*测试主板中左侧靠墙检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_LT_CR          = 0x0d, /*测试主板中右侧靠墙检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_LT_FL          = 0x0e, /*测试主板前左侧靠墙检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_LT_FR          = 0x0f, /*测试主板前右靠墙检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_WHEEL_DROP_L   = 0x10, /*测试主板左轮抬起检测是否正常。主板回复的数据，0：失败，1*/
    CMD_WHEEL_DROP_R   = 0x11, /*测试主板右轮抬起检测是否正常。主板回复的数据，0：失败，1:成功*/
    CMD_REMOTE         = 0x12, /*测试主板虚拟墙检测是否正常。主板回复的数据，0：失败，1：成功*/     
    CMD_BUOY_L         = 0x13, /*测试主板左目检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_BUOY_R         = 0x14, /*测试主板右目检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_BAT            = 0x15, /*测试主板电池电压是否正常。主板回复的数据：电池电压，单位mv。*/
    CMD_TEMP           = 0x16, /*测试主板电池温度是否正常。主板回复的数据：电池温度，单位摄氏度 */
    CMD_JACK_CHARGING  = 0x17, /*测试主板变压器充电检测是否正常。主板回复的数据，0：失败，  1：成功*/
    CMD_JACK_CURRENT   = 0x18, /*测试主板变压器充电电流是否正常。主板回复的数据，充电电流，单位ma*/
    CMD_DOCK_CHARGING  = 0x19, /*测试主板充电座充电检测是否正常。主板回复的数据，0：失败，1：成功*/
    CMD_DOCK_CURRENT   = 0x1a, /*测试主板充电座充电电流是否正常。主板回复的数据，充电电流，单位ma*/    
    CMD_WHEEL_L_FORWARD= 0x1B, /*测试主板左轮转动时的电流是否正常。主板回复的数据，电流，单位ma*/    
    CMD_WHEEL_R_FORWARD= 0x1C, /*测试主板右轮转动时的电流是否正常。主板回复的数据，电流，单位ma*/    
    CMD_WHEEL_L_REV_SPD= 0x1D, /*测试主板左轮转动的速度是否正常。主板回复的数据，轮子设定速度和实际速度的差值（绝对值）*/ 
    CMD_WHEEL_R_REV_SPD= 0x1E, /*测试主板右轮转动的速度是否正常。主板回复的数据，轮子设定速度和实际速度的差值（绝对值）*/
    CMD_MAIN_FORWARD   = 0x1f, /*测试主板主刷正向转动时的电流是否正常。主板回复的数据，电流，单位ma*/ 
    CMD_MAIN_REV       = 0x20, /*测试主板主刷反向转动时的电流是否正常。主板回复的数据，电流，单位ma*/ 
    CMD_SIDE_FORWARD   = 0x21, /*测试主板副刷正向转动时的电流是否正常。主板回复的数据，电流，单位ma*/ 
    CMD_SIDE_REV       = 0x22, /*测试主板副刷反向转动时的电流是否正常。主板回复的数据，电流，单位ma*/    
    CMD_VACCUM         = 0x23, /*测试主板吸尘的电流是否正常。主板回复的数据，电流，单位ma*/
    CMD_BINFULL        = 0x24, /*TURE OR FALSE*/
    CMD_DEBRIS         = 0x25, /*TURE OR FALSE*/
    CMD_OPEN_ALL_LEDS  = 0x26,
    CMD_UI_VERSION     = 0x27,
    CMD_WHEEL_SPEED    = 0x28,
    CMD_GYRO           = 0X29,
    CMD_FLASH_SONG     = 0x2A,
    CMD_UART           = 0x2B,
    CMD_MPLAYER        = 0x2C,
    CMD_MASTER_VERSION = 0x2D,
    CMD_TICK           = 0x2E,
    CMD_LONGTEST       = 0x2F,
    CMD_STASIS_TEST    = 0x30, /*万向轮，返回ture or FALSE*/
    CMD_TEST_ALL       = 0x31,
    CMD_BUOY_MIDDLE    = 0x32,
    CMD_TEST_CLIFF_LIGHTTOUCH  = 0x33,
    CMD_TEST_BUMP_DROP         = 0x34,
    CMD_TEST_ALL_IR            = 0x35,
    CMD_TEST_CLIFF_DROP_STASIS = 0x36,
    CMD_TEST_LIGHTTOUCH_BUMP   = 0x37,
    CMD_TEST_WIRELESS          = 0x38,
    CMD_TEST_MOVE_REVERSE      = 0x39,
    CMD_TEST_ALL_CHARGE_MODE   = 0x40,
//    CMD_DISABLE_SENSOR = 0xf0000024,
    CMD_NULL           = 0xf000008F	
}TEST_CMD_E;

/*状态命令类型*/
typedef enum
{
    CMD_TIME_INFO      = 0xf0000001, /*获取主板的rtc时间 tm*/
    CMD_SYS_STATE_INFO = 0xf0000002, /*获取主板系统状态 sys_state_info*/
    CMD_SYS_SCHEDULE_INFO=0xf0000004, /*获取预约时间表 schedule_time_info*/
    CMD_SYS_TEST_INFO    = 0xf0000008,
    CMD_SYS_ADC_INFO     =0xf0000010,  /*获取主板各个adc的值 sys_adc_info*/    
    CMD_GPIO_INFO        = 0xf0000020
}STATE_CMD_E;



typedef enum
{
  UART_CMD_NULL = 0x0000,
  UART_START = 0x0001,
  UART_STOP  = 0x0002,
  UART_FORWARD= 0x0004,
  UART_LEFT   = 0x0009,
  UART_RIGHT   = 0x0011,
  UART_BACK   = 0x0012,
  UART_DOCK  = 0x0020,
  UART_SPOT  = 0x0040,
  UART_WALL_FOLLOW = 0x60,
  UART_MODE1 = 0x0081,
  UART_MODE2 = 0x0082,
  UART_MODE3 = 0x0083,
  UART_MODE4 = 0x0084,
  UART_MODE5 = 0x0085,
  UART_MODE6 = 0x0086,
  UART_VOL_ADD = 0x1001,
  UART_VOL_SUB = 0x1002,
  UART_POWER_ON   = 0x1003,
  UART_POWER_DOWN = 0x1004,
  UART_SOS_ON     = 0x1005,
  UART_SOS_OFF    = 0x1006,
  UART_MPLAY_ON   = 0x1007,
  UART_MPLAY_OFF = 0x1008,
  UART_UV_ON      = 0x1009,
  UART_UV_OFF     = 0x100a,
  UART_SCHEDULE   = 0x100b,
  UART_OEM        = 0x100c,
  UART_GET_PRODUCT_ID = 0x100d,
  UART_GET_SCHDULE_INFO = 0x100e,
  UART_PLAY_MUSIC = 0x100f,
  UART_STOP_MUSIC = 0x1010,
  UART_ENHANCE    = 0X1011,
  UART_QUIT   = 0X1012
}UART_CTR_CMD_E;

#define UART_CMD_MAX 13
#endif

