#ifndef __DATA_BASE_H__
#define __DATA_BASE_H__

#include "am_type.h"

#pragma pack(1) // 1 bytes对齐

typedef enum
{
 ROBOT_STATE_OFF  = 0,
 ROBOT_STATE_WAITING =1,
 ROBOT_STATE_WALLFOLLOW=2, 
 ROBOT_STATE_CLEANING_ROOM =3,
 ROBOT_STATE_DOCK    =4,
 ROBOT_STATE_SPOTTING =5,
 ROBOT_STATE_REMOTE_DRIVE= 6,
 ROBOT_STATE_DEAD =7,
 ROBOT_STATE_TESTING= 8,
 ROBOT_STATE_CHARGING= 10,
 ROBOT_STATE_CHARGING_ERROR= 11,
 ROBOT_STATE_SLEEP=12,
 ROBOT_STATE_PAUSE =14,
}ROBOT_STATE_E;

/*charge_state*/
//#define CHARGING_STATE_OFF             0
//#define CHARGING_STATE_ON              5
//#define CHARGING_STATE_PURLING         7
//#define CHARGING_STATE_PURLING_STAGE2  8

#define IS_RUNNING_MODE(x) ( (x < ROBOT_STATE_DEAD ) & (x>ROBOT_STATE_WAITING) ) 
#define IS_CLEANNING_MODE(x) ( (x < ROBOT_STATE_REMOTE_DRIVE ) & (x>ROBOT_STATE_WAITING) ) 

/////////////////////////电源信息/////////////////////////////
/*  
 * power info  for bat
 */
typedef struct
{
    U16 bat_capacity_max;        /*电池容量*/	
    U16 bat_voltage_max;         /*正常工作最大电压*/
    U16 bat_voltage_min;         /*正常工作最小电压*/  
    U16 bat_voltage_dead;        /*截止工作电压*/       
    U16 bat_voltage_charge_max;  /*充电截止最大电压值*/
    U16 bat_temperature_max;     /*运行工作的最大温度值*/
    U16 wheel_crash_current;     /*判断碰撞障碍物左右轮子的最小电流*/  
    U16 wheel_twine_current;     /*判断左右轮缠绕最小电流*/  
    U16 main_brush_twine_current;/*判断主刷缠绕最小电流*/  
    U16 side_brush_twine_current;/*判断边刷轮缠绕最小电流*/ 
    U16 main_brush_voltage;  /*主刷的最大工作电压*/
    U16 side_brush_voltage;  /*边刷的最大工作电压*/
    U16 vacuum_voltage;      /*真空的最大工作电压*/
    U16 main_brush_reverse_voltage;/*主刷反转电压*/
    U16 side_brush_reverse_voltage;/*边刷反转电压*/
    U16 bat_voltage_adjust;       /*adc 校准值*/
    U16 rev1;
    U16 rev2;
    U16 rev3;
    U16 rev4;
    U16 rev5;
    U16 rev6;    
}power_info;
/////////////////////sensor信息///////////////////
/*  
 * all sensor info 
 */
typedef struct
{
    U8  cliff_en; /*是否使能cliff检测*/
    U8  lt_en;    /*是否使能lt检测*/
    U8  drop_en;  /*是否使能轮子跌落检测*/
    U8  bump_en;
    U8  rev1;
    U8  rev2;
}sensor_info;
//////////////////////硬件结构信息//////////////////////
/*  
 * hard structure info
 */
typedef struct
{
  U32 wheel_code_ratio;         /*码盘比例*/	  
  U16 wheel_dia;                /*左右轮子的直径*/
  U16 wheel_left_right_distance;/*轮子之间的距离*/
  U16 pad_width;                /*主刷清扫的有效长度*/
  U16 rev1;
  U16 rev2;
  U16 rev3;
  U16 rev4;
  U16 rev5;
  U16 rev6;  
}hard_structure_info;
//////////////////////行为信息/////////////////////////
/*描述行为的速度*/
typedef struct
{
  U16 clean_speed_max;  /*清扫的最大速度*/
  U16 crash_speed;      /*遇到障碍物降速速度*/
  U16 dock_speed_nomal; /*自座速度*/
  U16 dock_speed_low;   /*自座降速速度*/
  U16 rev1;
  U16 rev2;
  U16 rev3;
  U16 rev4; 
}act_speed_info;

/*描述行为的功能*/
typedef struct
{
  union{
    struct{	
      U16 main_brush_clean :1; /*配置扫地时是否关闭主刷, ON:open, OFF:close*/
      U16 main_brush_dock  :1; /*配置自座时是否关闭主刷, ON:open, OFF:close*/	  
      U16 side_brush_clean :1; /*配置扫地时是否关闭边刷, ON:open, OFF:close*/	  		  
      U16 side_brush_dock  :1; /*配置自座时是否关闭边刷, ON:open, OFF:close*/
      U16 vaccum_clean     :1; /*配置扫地时是否关闭真空, ON:open, OFF:close*/
      U16 vaccum_dock      :1; /*配置自座时是否关闭真空, ON:open, OFF:close*/	  
      U16 water_box        :1; /*配置自座时是否关闭真空, ON:open, OFF:close*/
      U16 resume           :1; /*配置主板支持断点续扫  ，ON:open, OFF:close*/
      U16 rev              :8;
    }BITS;
    U16 BYTE;
  }ACT_FUN;
}act_fun_info;

/*描述行为的数据结构*/
typedef struct
{
    act_speed_info act_speed_info_t;
    act_fun_info   act_fun_info_t;
}act_info;
////////////////////红外信息////////////////////////
/*  
 * remote info, the value use enum IR_E
 */
typedef struct
{
    U8 dock;    /*自座码值*/
    U8 clean;   /*清扫码值*/  
    U8 spot;    /*局部码值*/  
    U8 power;   /*关机码值*/  
    U8 left;    /*左向码值*/     
    U8 right;   /*右向码值*/  
    U8 forward; /*前向码值*/  
    U8 drivestop; /*停止码值*/  
    U8 backward;  /*向后码值*/
    U8 clock;
    U8 schedule;
    U8 wall_follow;
    U8 dirt;
    U8 random;
    U8 rev1;
    U8 rev2;
    U8 rev3;  
    U8 rev4;    
}remote_info;
/*  
 * dock ir info, the value use enum DOCK_E
 */
typedef struct
{
    U8  force_field; /*自座防撞区*/
    U8  buoy_red;    /*左目*/
    U8  buoy_green;  /*右目*/
    U8  buoy_both;   /*对准双目*/
    U8  rev1;
    U8  rev2;    
}dock_ir_info;
/*  
 * all ir info
 */
typedef struct
{
    remote_info    remote_info_t;
    dock_ir_info   dock_ir_info_t;
}ir_info;
///////////////////////end config info/////////////////////
////////////////////状态信息////////////////////
/*  
 * robot sys info, from main board
 */
typedef struct
{
    U8 robot_state;    /*运行状态*/
    U8 charging_state; /*充电状态*/
    U8 dock_state;     /*dock 状态*/
    U8 sound_state;    /*声音状态*/
    U8 flush_state;    /*闪烁标志*/
    U8 switch_state;   /*电源开关*/    
    U8 pause_id;       /*暂停原因*/
    U8 bin_full;       /*垃圾盒检测 1:垃圾满 0:未满*/
    U8 rev1;
    U8 rev2;
    U8 rev3;
    U8 rev4;
    U8 software_version; /*软件版本*/
}sys_state_info;


typedef struct
{
    U16 left_motor_current;
    U16 right_motor_current;
    U16 main_brush_current;
    U16 side_brush_current;
    U16 vac_current;
    U16 battery_voltage;
    S16 battery_current;
    S16 battery_thermistor; 
    U16 rev1;
    U16 rev2;    
}sys_current_info;

typedef struct 
{
    S16 bat_type;
    S16 adapter_output_vol;
    S16 adapter_output_current;     	
    S16 pa_gain;     	
    S16 robot_using_lt_bump;
    S16 lt_acc_mmps;
    S16 lt_bump_throd;
    S16 lt_bump_left_right_throd;
    S16 lt_max_light_value; 
    S16 rev1;
    S16 rev2;
    S16 rev3;
    S16 rev4;
    S16 rev5;
    S16 rev6;    
}externsion_info;

typedef struct
{  
    U16 adc_cliff_left;
    U16 adc_cliff_right;     
    U16 adc_cliff_front_left;
    U16 adc_cliff_front_right;
    U16 adc_cliff_rear_left;  
    U16 adc_cliff_rear_right;
    U16 adc_lighttouch_left;
    U16 adc_lighttouch_right; 
    U16 adc_lighttouch_front_left;   
    U16 adc_lighttouch_front_right;    
    U16 adc_lighttouch_center_left;
    U16 adc_lighttouch_center_right;
    U16 rev1;
    U16 rev2;
    U16 rev3;
    U16 rev4;
}sys_sensor_adc_info;
////////////////////////系统时间信息/////////////////////////
/*  
 * standard time struct, from main board
 */
typedef struct
{
    U8 hour;
    U8 min;
    U8 sec;
    //公历日月年周
    U16 w_year;
    U8  w_month;
    U8  w_date;
    U8  week;
}ui_tm;
/////////////////////////////schedule////////////////////////////////
/*  
 * schedule info, from UI
 *可以支持一次性最多10个的schdule信息
 *信息符合标准的tm结构，可以支持单次触发和循环触发
 *1 :tm年月日信息为0, 只带week, hour, min信息时为循环触发
 *2 :tm年月日信息不为0，为单次触发
 */
typedef struct
{
    union{
      struct{
       U8 sunBit  :1;
       U8 monBit  :1;
       U8 tueBit  :1;
       U8 wedBit  :1;
       U8 thuBit  :1;
       U8 friBit  :1;
       U8 satBit  :1;
       U8 rev     :1;
      }BITS;
      U8 BYTE;
    }SCH_ENABLE;	  
    ui_tm t[7];  
}schedule_time_info;

typedef struct 
{
    U16 start_x;
    U16 start_y;
    U16 number;
}map_request_info_t;

typedef enum
{
    MAP_POINT_TYPE_UNEXPLORED = 0,
    MAP_POINT_TYPE_FREEDOM = 1,
    MAP_POINT_TYPE_BUMPPED = 2,
    MAP_POINT_TYPE_CURRENT = 3,
}map_type_t;

#define MAP_POINTS_MAX 16
#define BITMAP_BUF_SIZE 26
typedef struct 
{
    U16 x;
    U16 y;
    U8 type;
}map_point_t;

typedef struct
{
    U16 direction;
    U8  point_count;
    map_point_t points[MAP_POINTS_MAX];
}t_map_info;

typedef struct
{
    t_map_info data;
}obj_map_info_t;


typedef struct
{
    U8 frame_h;
    U8 frame_l;
    U8 start_x;
    U8 start_y;
    U8 end_x;
    U8 end_y;
}cleaned_bitmap_param_t;

typedef struct
{
    U8 err_map_data_flag;
    U8 update_his_map_flag ;
    cleaned_bitmap_param_t param;
    U8 bitdata[BITMAP_BUF_SIZE];
}bitmap_info_t;
typedef struct
{
  U32 cmd;
  U8  cmd_type;
  U8  cmd_state;
  U8  len;
}command;

typedef struct
{
  int    angle;
  float  dist;
  int    rev1;
  int    rev2;
}direction_key_info;

typedef struct
{
    U8 fc;    
    U8 id; 
    command info;     
    U8 data;     
}obj_command_info_t;


typedef struct
{
    U8 fc;    
    U8 id; 
    command info;     
    sys_state_info data;     
}obj_sys_state_info_t;
//////////////////////////////////////////////////////////////
#pragma pack ()
#endif