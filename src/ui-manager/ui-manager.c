//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "ui-manager.h"
#include "ui-song-player.h"
#include "ui-msg-handler.h"
#include "syscall_api.h"
#include "docking-new.h"
#include "dock-avoid.h"
#include "virtual-wall.h"
#include "sensor/sensor.h"
#include "charge/charge.h"
#include "am_uart_api.h"
#include "am_exti_cfg.h"
#include "am_spi_cfg.h"
#include "am_tm_api.h"
#include "motor/robot_suction.h"
#include "motor/robot_brush.h"
#include "monitor/robot_batter.h"
#include "ui-test_cmd_handle.h"
#include "monitor/robot_univrsal_wheel.h"
#include "am_config_macro.h"
//from ui-config.h
extern void ui_config_init(void);
extern void set_ui_config_update_callback(ui_config_update_func_t func);
extern ui_config_t* get_ui_config(void);
extern void ui_config_update(void);
extern void ui_put_map_point_info(uint16_t x, uint16_t y, uint8_t type, uint16_t direction);

static int ui_manager_init(void);     //initial
static int ui_manager_deinit(void);  //deinitial

static ui_config_t* ui_manager_get_config(void);
static int ui_manager_put_massage(int msg_id, int arg, void* data);
static int ui_manager_put_error_info(int errno, int arg);
static int ui_manager_clear_error_info(void);

static int ui_manager_put_sensor_data (int sensor_type, int index, void* data);
static int ui_manager_put_real_map_point (int x, int y, int type, int angel);
static int ui_manager_quit_song(void);

/*下面結構體函數指針必須有對應實體函數賦值，那怕函數是空實現lyy*/
const static ui_manager_t ui_manager = {
  .init = ui_manager_init,
  .deinit = ui_manager_deinit,
  .get_config = ui_manager_get_config,
  .put_massage = ui_manager_put_massage,
  .put_error_info = ui_manager_put_error_info,
  .clear_error_info = ui_manager_clear_error_info,
  .put_sensor_data = ui_manager_put_sensor_data,
  .put_real_map_point = ui_manager_put_real_map_point,
  .quit_song = ui_manager_quit_song,
  .dock_new_init = dock_new_init,
  .dock_new_start = dock_new_start,
  .dock_new_end = dock_new_end,
  .set_dock_new_end = set_dock_new_end,
  .set_lt_enable    = set_lighttouch_enable,
  .set_cliff_enable = set_cliff_enable,
  .robot_is_cliff  = robot_is_cliff,
  .robot_signal_distance= robot_signal_distance,
  .robot_is_lighttouch =  robot_is_lighttouch,
  .robot_rear_lt_distance = robot_rear_lt_distance,//一般方案没有这组
  .charging_init     = charging_init,
  .process_charging  = process_charging,
  .quit_charging     = quit_charging,
  .charging_state    = charging_state_get,
  .sys_tm_cfg        = sys_tm_cfg,
  .sys_tm_close      = sys_tm_close,
  .sys_set_pwm_out_duty = sys_set_pwm_out_duty,
  .sys_uart_cfg = sys_uart_cfg,
  .sys_uart_close = sys_uart_close,
  .exit_wakeup_cfg = exit_wakeup_cfg,
  .wakeup_pin_cfg  = wakeup_pin_cfg,
  .sys_spi_cfg     = sys_spi_cfg,
  .sys_spi_close   = sys_spi_close,
  .robot_suction_set   =  robot_suction_vols_set,
  .robot_sidebrush_set =  robot_sidebrush_vols_set,
  .robot_midbrush_set  =  robot_midbrush_vols_set,
  .avoid_active = dock_avoid_active,
  .avoid_signal_quality = dock_avoid_signal_quality,
  .binocular_avoid_signal_quality = dock_binocular_avoid_signal_quality,
  .battery_is_low                 = battery_is_low,
  .virtual_wall_active            = virtual_wall_active,
  .get_bump_state      = robot_bump_mask ,
  .get_wheelDrop_state = robot_wheel_drop_mask,
  .turn_off_touch_bump = turn_off_touch_bump,
  .turn_on_touch_bump  = turn_on_touch_bump,
  .touch_bump_run      = touch_bump_run,
  .universal_wheel_monitor = universal_wheel_monitor,
  .universal_wheel_count   = universal_wheel_count,
  .robot_universal_wheel_init = robot_universal_wheel_init,
  .wall_follow_callback = wall_follow_callback, //默认用sdk内部的wallfollow
  .wall_follow_start    = wall_follow_start,//默认用sdk内部的wallfollow
  .wall_follow_stop     = wall_follow_stop,  //默认用sdk内部的wallfollow
  .set_cliff_threshold  = set_cliff_threshold,
  .reset_cliff_threshold= reset_cliff_threshold,
  .charging_jack      = charging_jack,
  .charging_dock      = charging_dock, 
  .convert_to_voltage = convert_to_voltage,
  .convert_to_current = convert_to_current,
};

const ui_manager_t* get_ui_manager(void)
{
  return &ui_manager;
}

static int ui_manager_init(void)
{
  ui_config_init();
  return 0;
}
static int ui_manager_deinit(void)
{
  return 0;
}

static  ui_config_t* ui_manager_get_config(void)
{
  return (ui_config_t*)get_ui_config();
}


static int ui_manager_put_massage(int msg_id, int arg, void* data)
{
  return ui_msg_handle((UiMsgType)msg_id, arg, data);
}

static int ui_manager_put_error_info(int errno, int arg)
{
  return 0;//system_error_handle(errno, arg);
}

static int ui_manager_clear_error_info(void)
{
  return 0;
}

static int ui_manager_put_sensor_data (int sensor_type, int index, void* data)
{
  return 0;
}


static int ui_manager_put_real_map_point (int x, int y, int type, int angel)
{
   #if defined(USE_WIFI_DEMO_1) || defined(USE_WIFI_DEMO_2)
   ui_put_map_point_info(x,y,type,angel); 
   #endif
   return 0;
}

static int ui_manager_quit_song(void)
{
  return 0;
}