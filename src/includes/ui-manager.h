#ifndef __UI_MANAGER_H
#define __UI_MANAGER_H

#include "ui-config.h"
#include "docking-core.h"
#include "ui-test_cmd_handle.h"
#include "am_robot_type.h"
#include "wall_follow/wall_follow_common.h"
#include "sensor/sensor.h"
typedef enum
{
    UI_MSG_BUTTON_REJECT,

    UI_MSG_MAP_SERCHING_START,
    UI_MSG_MAP_SERCHING_STOP,

    UI_MSG_SLEEPING_ENTER,
    UI_MSG_SLEEPING_EXIT,

    UI_MSG_POWER_OFF_ENTER,
    UI_MSG_POWER_UP_ENTER,
    UI_MSG_POWER_UP_OK,
    UI_MSG_POWER_UP_CHARGING,
    UI_MSG_POWER_UP_BATTLOW,

    UI_MSG_SELF_DIAGNOSTICS_ENTER,
    UI_MSG_SELF_DIAGNOSTICS_EXIT,

    UI_MSG_CLEANING_NOT_READY,
    UI_MSG_CLEANING_START,
    UI_MSG_CLEANING_PAUSE,
    UI_MSG_CLEANING_RESUME,
    UI_MSG_CLEANING_RESET,
    UI_MSG_CLEANING_ERROR_STOP,
    UI_MSG_CLEANING_PICKUP_STOP,
    UI_MSG_CLEANING_USER_STOP,
    UI_MSG_CLEANING_COMPLETE,

    UI_MSG_CRITICAL_LOW_POWER,
    UI_MSG_RESERVE_POWER,
    UI_MSG_BATT_LOW,
    UI_MSG_BATT_SUFF,
    UI_MSG_BATT_ALMOST_FULL,
    UI_MSG_BATT_FULL,
    UI_MSG_BATT_LEVEL_CHANGED,

    UI_MSG_CHARGING_START,
    UI_MSG_CHARGING_STARTED,
    UI_MSG_CHARGING_STOPPED,
    UI_MSG_CHARGING_ERROR,
    UI_MSG_CHARGING_COMPLETE,

    UI_MSG_DOCKING_START,
    UI_MSG_DOCKING_STARTED,
    UI_MSG_DOCKING_STOPPED,
    UI_MSG_DOCKING_COMPLETE,

    UI_MSG_HOMING_START,
    UI_MSG_HOMING_STARTED,
    UI_MSG_HOMING_STOPPED,
    UI_MSG_HOMING_COMPLETE,

    UI_MSG_NORMAL_CLEANING_START,
    UI_MSG_NORMAL_CLEANING_STARTED,
    UI_MSG_NORMAL_CLEANING_STOPPED,
    UI_MSG_NORMAL_CLEANING_COMPLETE,

    UI_MSG_SPOT_CLEANING_START,
    UI_MSG_SPOT_CLEANING_STARTED,
    UI_MSG_SPOT_CLEANING_STOPPED,
    UI_MSG_SPOT_CLEANING_COMPLETE,

    UI_MSG_WALLFOLLOW_CLEANING_START,
    UI_MSG_WALLFOLLOW_CLEANING_STARTED,
    UI_MSG_WALLFOLLOW_CLEANING_STOPPED,
    UI_MSG_WALLFOLLOW_CLEANING_COMPLETE,

} UiMsgType;

typedef enum
{
    UI_ERROR_00_REVERSE               = 0,
    UI_ERROR_01_LEFT_WHEEL_HANGED     = 1,
    UI_ERROR_02_MAIN_BRUSH_ENTWINED   = 2,
    UI_ERROR_03_RIGHT_WHEEL_HANGED    = 3,
    UI_ERROR_04_LEFT_WHEEL_ENTWINED   = 4,
    UI_ERROR_05_RIGHT_WHEEL_ENTWINED  = 5,
    UI_ERROR_06_ALWAYS_CLIFFED        = 6,
    UI_ERROR_07_SIDE_BRUSH_ENTWINED   = 7,
    UI_ERROR_08_CODEPANEL_ERROR       = 8,
    UI_ERROR_09_BUMP_ERROR            = 9,
    UI_ERROR_10_DUSTBIN_UN_INSERT     = 10,
    UI_ERROR_11_STASIS_STALL          = 11,
    UI_ERROR_12_CLIFF_ERROR           = 12,
    UI_ERROR_13_REVERSE               = 13,
    UI_ERROR_14_REVERSE               = 14,
    UI_ERROR_15_VACCUM_MOTO_ENTWINED  = 15,
    UI_ERROR_16_BIG_CURRENT           = 16,
    UI_ERROR_17_SWITCH_OFF_CHARGING   = 17,
    UI_ERROR_18_REVERSE               = 18,
    UI_ERROR_NUMBER_MAX,
} UI_ERROR_NUMBERS_E;

typedef enum
{
    SENSOR_TYPE_KEY,
    SENSOR_TYPE_IR_REMOTE,
    SENSOR_TYPE_RF_REMOTE,
    SENSOR_TYPE_WIFI_REMOTE,
    SENSOR_TYPE_LIGHTTOUCH_SENSOR,
    SENSOR_TYPE_CLIFF_SENSOR,
    SENSOR_TYPE_DROP_SENSOR,
    SENSOR_TYPE_UART,
} sensor_type_e;


typedef void  (*fun_t)(void);
typedef int (*system_call_func_t) (int cmd, int arg1, int arg2, void* data);
typedef int (*ui_manager_init_t) (void);
typedef int (*ui_manager_deinit_t) (void);
typedef int (*ui_manager_put_massage_t) (int msg_id, int arg, void* data);
typedef int (*ui_manager_put_error_info_t) (int errno, int arg);
typedef int (*ui_manager_clear_error_info_t) (void);
typedef ui_config_t* (*ui_manager_get_config_t) (void);
typedef int (*ui_manager_put_sensor_data_t) (int sensor_type, int index, void* data);
typedef int (*ui_manager_put_real_map_point_t) (int x, int y, int type, int angel);
typedef int (*ui_manager_quit_song_t) (void);
typedef dock_config_t* (*dock_new_init_t)(void);
typedef void (*dock_new_start_t)(void);
typedef BOOLEAN (*dock_new_end_t)(U8 *uTerm);
typedef void (*set_dock_new_end_t)(void);

typedef U8  (*robot_is_cliff_t)(U8 index);
typedef S16 (*robot_signal_distance_t)(U8 index);
typedef u8  (*robot_is_lighttouch_t)(u8 index);
typedef void (* lt_on_off_set_t)(u8 en);
typedef void (* cliff_on_off_set_t)(u8 en);
typedef void (* reset_lt_auto_offset_t)(void);
typedef S16  (*robot_rear_lt_distance_t)(U8 index);
typedef void (*charging_init_t)(void);
typedef void (*process_charging_t)(void);
typedef void (*quit_charging_t)(void);
typedef u16  (*charging_state_t)(void);

typedef void (*sys_tm_cfg_t)(void);
typedef void (*sys_tm_close_t)(void);
typedef void (*sys_set_pwm_out_duty_t)(s16 channel , s16 Dutyfactor);

typedef void (*sys_uart_cfg_t)(void);
typedef void (*sys_uart_close_t)(void);

typedef s16 (*exit_wakeup_cfg_t)(void);
typedef s16 (*wakeup_pin_cfg_t)(void);

typedef void (*sys_spi_cfg_t)(void);
typedef void (*sys_spi_close_t)(void);

typedef void (*robot_suction_set_t)(u16 speed);
typedef void (*robot_sidebrush_set_t)(u16 speed);
typedef void (*robot_midbrush_set_t)(u16 speed);

typedef BOOLEAN (*dock_avoid_active_t)(BOOLEAN min);
typedef U8 (*dock_avoid_signal_quality_t)(void);
typedef U8 (*dock_binocular_avoid_signal_quality_t)(void);

typedef BOOLEAN (*virtual_wall_active_t)(void);

typedef U8 (*battery_is_low_t)(void);

typedef BumpState        (*get_bump_mask_t)(void);
typedef WheelDropState   (*get_wheelDrop_state_t)(void);
typedef void (*turn_off_touch_bump_t)(void);
typedef void (*turn_on_touch_bump_t)(void);
typedef int (*touch_bump_run_t)(void);
typedef int (*universal_wheel_monitor_t)(void *arg);
typedef int (*universal_wheel_count_t)(void);
typedef void (*robot_universal_wheel_init_t)(void);
typedef int (*wall_follow_callback_t)(WfParames_t *);
typedef int (*wall_follow_start_t)(AM_LeftRight contact_side, WfParames_t* wfp_p);
typedef void (*wall_follow_stop_t)(WfParames_t* wfp_p);
typedef void (*set_cliff_threshold_t)(SENSOR_E chan, int val);
typedef void (*reset_cliff_threshold_t)(void);
typedef U16  (*charging_jack_t)(void);
typedef U16  (*charging_dock_t)(void);
typedef s16  (*convert_to_voltage_t)( s16 v_adc);
typedef s16  (*convert_to_current_t)( s16 c_adc);
/*下面是UI 必须要实现提供给系统调用的函数，lyy */
typedef struct ui_manager_t
{
   //初始化函数，系统得到此结构后需要先调用此函数
    ui_manager_init_t init;
    //关机时调用此函数
    ui_manager_deinit_t deinit;

    //系统有消息需要通知UI时调用此函数
    ui_manager_put_massage_t put_massage;

    //取得整个UI配置，随时可调用。
    ui_manager_get_config_t get_config;

    //系统有故障发生时，调用此函数。
    ui_manager_put_error_info_t put_error_info;

    ui_manager_clear_error_info_t clear_error_info;

    ui_manager_put_sensor_data_t put_sensor_data;

    ui_manager_put_real_map_point_t put_real_map_point;

    ui_manager_quit_song_t quit_song;
    //dock 提供给系统调用的函数
    dock_new_init_t    dock_new_init;

    dock_new_start_t   dock_new_start;

    dock_new_end_t     dock_new_end;

    set_dock_new_end_t set_dock_new_end;
    //关闭off
    lt_on_off_set_t    set_lt_enable;
    cliff_on_off_set_t set_cliff_enable;

    robot_is_cliff_t   robot_is_cliff;
    //cliff , light touch
    //这个计算结果，需要提供给系统用
    robot_signal_distance_t  robot_signal_distance;
    robot_rear_lt_distance_t robot_rear_lt_distance;
    //这个函数提供给系统调用
    robot_is_lighttouch_t  robot_is_lighttouch;
    //charging
    charging_init_t        charging_init;
    process_charging_t     process_charging;
    quit_charging_t        quit_charging;
    charging_state_t       charging_state;
    //driver
    sys_tm_cfg_t           sys_tm_cfg;
    sys_tm_close_t         sys_tm_close;
    sys_set_pwm_out_duty_t sys_set_pwm_out_duty;
    sys_uart_cfg_t         sys_uart_cfg;
    sys_uart_close_t       sys_uart_close;
    exit_wakeup_cfg_t      exit_wakeup_cfg;
    wakeup_pin_cfg_t       wakeup_pin_cfg;
    sys_spi_cfg_t          sys_spi_cfg;
    sys_spi_close_t        sys_spi_close;
    robot_suction_set_t    robot_suction_set;
    robot_sidebrush_set_t  robot_sidebrush_set;
    robot_midbrush_set_t   robot_midbrush_set;
    dock_avoid_active_t    avoid_active;
    dock_avoid_signal_quality_t avoid_signal_quality;
    dock_binocular_avoid_signal_quality_t binocular_avoid_signal_quality;
    battery_is_low_t battery_is_low;
    virtual_wall_active_t virtual_wall_active;
    get_bump_mask_t        get_bump_state;
    get_wheelDrop_state_t  get_wheelDrop_state;
    turn_off_touch_bump_t  turn_off_touch_bump;
    turn_on_touch_bump_t   turn_on_touch_bump;
    touch_bump_run_t       touch_bump_run;
    universal_wheel_monitor_t universal_wheel_monitor;
    universal_wheel_count_t   universal_wheel_count;
    robot_universal_wheel_init_t robot_universal_wheel_init;
    wall_follow_callback_t  wall_follow_callback;
    wall_follow_start_t     wall_follow_start;
    wall_follow_stop_t      wall_follow_stop;
    set_cliff_threshold_t   set_cliff_threshold;
    reset_cliff_threshold_t reset_cliff_threshold;
    charging_jack_t         charging_jack;
    charging_dock_t         charging_dock;
    convert_to_voltage_t   convert_to_voltage;
    convert_to_current_t   convert_to_current;
    fun_t rev27;    
    fun_t rev28;
    fun_t rev29;
    fun_t rev30;
}ui_manager_t;

extern system_call_func_t system_call;
const ui_manager_t* get_ui_manager(void);
extern system_call_func_t system_call;


#endif /* __UI_MANAGER_H*/