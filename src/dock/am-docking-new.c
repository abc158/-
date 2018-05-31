//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "am-docking-new.h"
#include "am-docking-sensors.h"
#include "sensor/sensor.h"
#ifdef IR_WIRELESS
#include "dock_ir_signal.h"
#include "wireless/arf2496k.h"
#endif
#include "monitor/robot_batter.h"
#include "motor/robot_brush.h"

#ifdef AM_DOCKING_METHOD

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define  DOCK_WHERE_HAS_CONDITION 0X01<<0
#define  DOCK_WHERE_HAS_CENTER 0X01<<1

#define DOCK_NEW_DEBUG
#ifdef DOCK_NEW_DEBUG
enum {
	DEBUG_DOCK_BEHAVIOR = 0x1 << 0,
	DEBUG_DVRIER_GO = 0x1 << 1,
	DEBUG_TURN = 0x1 << 2,
	DEBUG_DOCK_ANGLE = 0x1 << 3,
	DEBUG_DOCK_FORCE_FIELD = 0x1 << 4,
};

static U8 debug_mask = 0;
#define dprintf(level_mask, msg...)  if(debug_mask&level_mask)\
	printf(msg)
#else
#define dprintf(msg...) {}
#endif
#if IR_LOCAL_IS_STEREO
#define DOCKING_TRUN_SLOWEST_SPEED    (120)
#define DOCKING_TRUN_SLOW_SPEED       (240)
#define DOCKING_NEAR_SLOWEST_SPEED    (50)//60//70
#define DOCKING_NEAR_SLOW_SPEED       (80)//70//100
#define DOCKING_SLOWEST_SPEED         (110)//70//110
#define DOCKING_SLOW_SPEED            (180)//125//180
#define FORWARD_NEAR_SLOW_SPEED       (60)//63//90
#define FORWARD_SLOW_SPEED            (140)//103//180
#define FORWARDSPEED                  (290)
#define BACKSPEED                     (80)//80//240

#define LINE_DOCK_TRIGGER_OFF         (40)
#define NEAR_LINE_DOCK_TRIGGER_OFF    (60)
#else
#define DOCKING_TRUN_SLOWEST_SPEED    (120)
#define DOCKING_TRUN_SLOW_SPEED       (240)
#define DOCKING_NEAR_SLOWEST_SPEED    (50)//60//70
#define DOCKING_NEAR_SLOW_SPEED       (80)//70//100
#define DOCKING_SLOWEST_SPEED         (110)//70//110
#define DOCKING_SLOW_SPEED            (180)//125//180
#define FORWARD_NEAR_SLOW_SPEED       (60)//63//90
#define FORWARD_SLOW_SPEED            (140)//103//180
#define FORWARDSPEED                  (290)
#define BACKSPEED                     (80)//80//240

#define LINE_DOCK_TRIGGER_OFF         (40)
#define NEAR_LINE_DOCK_TRIGGER_OFF    (40)
#endif


#define VERIFY_HOLD_CNT               (2)

#define CARE_BUMP                     (1)
#define CARE_CLIFF                    (2)
#define SIDE_BRUSH_SPEED              (330)

static DockingState docking_state =
{
	0,
	FALSE,
	0,
	0,
	0,
	0,
	0x0FFF,
	FALSE,
	FALSE,
};

static dock_config_t dock_config;
static AM_Pose 	dock_start_pose;
static BOOLEAN docking_dock_where_abort = FALSE;
extern float ir_receiver_distributions(int which);
extern float ir_receiver_angles_range(int which);


BOOLEAN docking_left_run_when(void);
BOOLEAN docking_right_run_when(void);

#ifdef IR_WIRELESS
void dock_wireless_rx_code_get(U8 chan, U8 val)
{
	docking_state.wireless_data=val;
}

void clear_dock_wireless_rx_code(void)
{
	docking_state.wireless_data=0;
}

U8 get_wireless_rx_code(void)
{
	return docking_state.wireless_data;
}
#endif

void set_dock_leave_dockagain(U8 state)
{
	//dock_config.dock_again = state;
	return;
}

dock_config_t *get_dock_config(void)
{
	return &dock_config;
}

static void docking_parameter_init(void)
{
	docking_state.dock_finished = FALSE;
	docking_state.dock_failured = NULL;
	docking_state.random_behavior_count = 0;
	docking_state.avoid_obstacle_count = 0;
	docking_state.dock_failure_count = 0;
	docking_state.state_cnt = 0;
	docking_state.near_dock = FALSE;
	docking_state.dock_angle = 0x0FFF;
    docking_state.dock_start_flag = TRUE;
#ifdef IR_WIRELESS
	docking_state.wireless_data = 0;
#endif
	return;
}


void clear_specify_debouncer(Debouncer_Data * _dd)
{
  if (_dd->current_state == TRUE)
  {
    if (_dd->set_dock_context != NULL)
      (* (_dd->set_dock_context))(FALSE);
  }
  _dd->on_count = 0;
  _dd->current_state = FALSE;
  _dd->off_count = 0;
  return;
}

void set_line_dock_trigger_off(U16 val)
{
    recently_docking_left.trigger_off = val;
    recently_docking_right.trigger_off = val;
    recently_docking_go_forward.trigger_off = val;
    return;
}

void am_go_to_place(S16 rotation_angle, S16 l_speed, S16 r_speed, BOOLEAN condition, int care_bump, S8* result) {
    S8 res;
    AM_GO_TO_PLACE(rotation_angle,l_speed,r_speed,condition,care_bump,res);
    *result = res;
}
void drive_go(S16 distance, S16 speed, BOOLEAN condition, int care_bump, S8* result) {
    S8 res;
    DRIVE_GO(distance,speed,condition,care_bump,res);
    *result = res;
}
/*********************************** DOCK SUCCESS **********************************/
/**
 * dock success - 判断上是否上座成功
 * 触发条件: 当接触片接触上时触发
 * 退出条件: 上座成功或失败
 */
static BOOLEAN docking_success_abort = FALSE;
void set_docking_success_abort(void)
{
	docking_success_abort = TRUE;

	return;
}

BOOLEAN docking_success_abort_when(void)
{
	if(docking_success_abort != FALSE)
		return TRUE;
	else
		return FALSE;
}

void docking_success_abort_code(void)
{

	docking_success_abort = FALSE;
	return;
}

DOCK_FN_DECL(docking_success)
{
  S8 result = 0;
  S16 vl_meas, vr_meas;
  
  dprintf(DEBUG_DOCK_BEHAVIOR, "docking_success\r\n");
  
  robot_sidebrush_vols_set(0);
  do
  {
    set_motor_vels(0, 0, ACCELERATION_MAX);
    get_motor_speeds(&vl_meas, &vr_meas);
  }
  while ((vl_meas > 0) || (vr_meas > 0));
  docking_state.state_cnt++;
  
  if (!charging_detect())
  {
    dprintf(DEBUG_DOCK_BEHAVIOR, "docking_verify_charger fail \r\n");
    docking_state.state_cnt = 0;
    robot_sidebrush_vols_set(SIDE_BRUSH_MAX_VOLTAGE/2); 
    DRIVE_GO(-200,BACKSPEED,(!charging_detect()),0,result);
    if(charging_detect())
      docking_state.dock_finished = TRUE;
    else
    {
      set_docking_success_abort();
    }
    return ;
  }
  else if (docking_state.state_cnt > VERIFY_HOLD_CNT)
  {
    set_motor_vels(0, 0, ACCELERATION_MAX);
    
    // we are really charging!
    dprintf(DEBUG_DOCK_BEHAVIOR, "docking_verify_charger ok \r\n");
    
    docking_state.dock_finished = TRUE;
    return ;
  }
  
  return ;
}

BOOLEAN docking_success_start_when(void)
{
	if (charging_detect() && (current_dock_behavior() != DOCKING_SUCCESS))
	{
		return TRUE;
	}
	else
		return FALSE;
}

void dock_success_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_SUCCESS;
	dock_funtion.start_when = &docking_success_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_success_abort_when;
	dock_funtion.abort_code = &docking_success_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_success;

	register_dock_function(&dock_funtion);

	return;
}
/********************************* DOCK SUCCESS END *******************************/


/*********************************** DOCK BOUNCE **********************************/
/**
 * dock bounce - 正对着充电座上座时的碰撞处理
 * 触发条件: 正对着充电座时，发生bump或cliff
 * 退出条件: 无
 */
static BOOLEAN docking_bounce_abort = FALSE;
static BOOLEAN docking_bounce_force_field = FALSE;

void set_docking_bounce_abort(void)
{
	docking_bounce_abort = TRUE;

	return;
}

BOOLEAN docking_bounce_abort_when(void)
{

	if(docking_bounce_abort != FALSE)
		return TRUE;
	else
		return FALSE;
}

void docking_bounce_abort_code(void)
{

	docking_bounce_abort = FALSE;

	return;
}

DOCK_FN_DECL(docking_bounce)
{
  S8 result = 0;
  S16 angle =170;
  uint32_t start_time;
  dprintf(DEBUG_DOCK_BEHAVIOR, "docking_bounce\r\n");
  
  if(get_bump_state()!=0)
    printf("dock_bump!!\r\n");
  if(get_cliff_state()!=0)
    printf("dock_cliff!!\r\n");
  
  while((get_bump_state() !=0) || (get_cliff_state() != 0)){};
  robot_sidebrush_vols_set(SIDE_BRUSH_MAX_VOLTAGE/2); 
  start_time=timer_ms();
  if(recently_docking_go_forward.current_state)
  {
    DRIVE_GO(-150,BACKSPEED,((!charging_detect())||(timer_elapsed(start_time)<500)),(CARE_CLIFF),result);
     if(recently_docking_go_forward.current_state)
    docking_state.dock_failure_count++;
     else
        docking_state.dock_failure_count+=3;
  }
  else
  {
    DRIVE_GO(-80,BACKSPEED,((!charging_detect())||(timer_elapsed(start_time)<500)),(CARE_CLIFF),result);
    //docking_bounce_force_field = TRUE;
    docking_state.dock_failured= DOCKING_FAIL_FORCE_FIEDL;
    printf("[dock_failure]docking_force_field\r\n");
  }
  
  set_docking_bounce_abort();
  
  return ;
}

BOOLEAN docking_bounce_start_when(void)
{
	if (((get_bump_state() !=0) || (get_cliff_state() != 0)) && \
		recently_near_dock.current_state)
	{
		return TRUE;
	}
	else
		return FALSE;
}

void docking_bounce_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_BOUNCE;
	dock_funtion.start_when = &docking_bounce_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_bounce_abort_when;
	dock_funtion.abort_code = &docking_bounce_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_bounce;

	register_dock_function(&dock_funtion);

	return;
}
/********************************* DOCK BOUNCE END ********************************/


/******************************* DOCK AVOID OBSTACLE ******************************/
/**
 * docking_avoid_obstacle - 避开障碍物
 * NOTE:在正对着座子上座时，遇到障碍物避开障碍物
 * 触发条件: 不在充电座附近，正对着座子上座时，发生bump或cliff
 * 退出条件: 无
 */
static BOOLEAN docking_avoid_obstacle_abort = FALSE;
static S16 docking_avoid_obstacle_angle = 0;

void set_docking_avoid_obstacle_abort(void)
{
	docking_avoid_obstacle_abort = TRUE;
	return;
}

BOOLEAN docking_avoid_obstacle_abort_when(void)
{

	if(docking_avoid_obstacle_abort != FALSE)
		return TRUE;
	else
		return FALSE;
}

void docking_avoid_obstacle_abort_code(void)
{
	docking_avoid_obstacle_abort = FALSE;
	return;
}

DOCK_FN_DECL(docking_avoid_obstacle)
{
	S8 result = 0;
	
	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_avoid_obstacle\r\n");

	while((get_cliff_state() != 0) || (get_bump_state() != 0)){};
    
	drive_go(-50,BACKSPEED,TRUE,CARE_CLIFF,&result);
	
	am_go_to_place(docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,&result);

	drive_go(100,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),&result);
	while((get_cliff_state() != 0) || (get_bump_state() != 0)){};
	
	am_go_to_place(-docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,&result);

	drive_go(200,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),&result);
	while((get_cliff_state() != 0) || (get_bump_state() != 0)){};

	am_go_to_place(-docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,&result);
	
	drive_go(100,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),&result);
	while((get_cliff_state() != 0) || (get_bump_state() != 0)){};
	printf("avoid_obstacle:%d!!!\n",docking_avoid_obstacle_angle);
	am_go_to_place(docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,0,&result);
	printf("avoid_obstacle:ok ret:%d!!!\n",result);
	//set_near_dock_context(FALSE);
	//docking_state.avoid_obstacle_count++;	
	set_docking_avoid_obstacle_abort();

	return ;
}

BOOLEAN docking_avoid_obstacle_start_when(void)
{
	BumpState bumped_state = get_bump_state();
	CliffState cliffed_state = get_cliff_state();

	if (((bumped_state !=0) || (cliffed_state != 0)) && \
		(!docking_bounce_start_when()) && \
		((current_dock_behavior() == DOCKING_LEFT) || \
		(current_dock_behavior() == DOCKING_RIGHT) || \
		(current_dock_behavior() == DOCKING_CORRECT) || \
		(current_dock_behavior() == DOCKING_GO_FORWARD)))
	{		
		if(get_bump_state()!=0)
			printf("dock_bump!!\r\n");
		if(get_cliff_state()!=0)
			printf("dock_cliff!!\r\n");
		if ((bumped_state & BUMP_FRONT_LEFT) || \
			(cliffed_state & CLIFF_SIDE_LEFT))
		{
			docking_avoid_obstacle_angle = -90;
		}
		else if ((bumped_state & BUMP_FRONT_RIGHT) || \
			(cliffed_state & CLIFF_SIDE_RIGHT))
		{
			docking_avoid_obstacle_angle = 90;
		}
		else if((bumped_state & BUMP_FRONT_CENTER) || \
			((cliffed_state & CLIFF_FRONT_LEFT) || \
			(cliffed_state & CLIFF_FRONT_RIGHT)))
		{
			if (docking_avoid_obstacle_angle != 0)
				docking_avoid_obstacle_angle = -docking_avoid_obstacle_angle;
			else
				docking_avoid_obstacle_angle = 90;
		}

		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void docking_avoid_obstacle_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_AVOID_OBSTACLE;
	dock_funtion.start_when = &docking_avoid_obstacle_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_avoid_obstacle_abort_when;
	dock_funtion.abort_code = &docking_avoid_obstacle_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_avoid_obstacle;

	register_dock_function(&dock_funtion);

	return;
}
/***************************** DOCK AVOID OBSTACLE END ****************************/


/************************************ DOCK RIGHT **********************************/
/**
 * dock right - 右摆行为
 * NOTE:
 * 触发条件: 当中间接收头收到F8信号时，触发
 * 退出条件: 无
 */
DOCK_FN_DECL(docking_right)
{
	TransVel  left_vel;
	TransVel  right_vel;
	BOOLEAN already_mid = FALSE;
	S16 right_start_angle = 0;
	S16 angle = 0;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_right\r\n");

while(1)
{
	if (already_mid == FALSE)
	{
		if (recently_near_dock.current_state)
		{
			left_vel = DOCKING_NEAR_SLOW_SPEED;
			right_vel = DOCKING_NEAR_SLOWEST_SPEED;
		}
		else
		{
			left_vel = DOCKING_SLOW_SPEED;
			right_vel = DOCKING_SLOWEST_SPEED;
		}
	}
	else
	{
		left_vel = 0;
		right_vel = 0;
	}

	set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
}
	return ;
}

BOOLEAN docking_right_run_when(void)
{
	if(!docking_dock_where_abort)
		return FALSE;

	if(recently_docking_right.current_state&&\
		current_dock_behavior()!=DOCKING_LEFT)
          return TRUE;
	else
		return FALSE;
}

void dock_right_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_RIGHT;
	dock_funtion.start_when = NULL;
	dock_funtion.run_when = &docking_right_run_when;
	dock_funtion.abort_when = NULL;
	dock_funtion.abort_code = NULL;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_right;

	register_dock_function(&dock_funtion);

	return;
}
/********************************** DOCK RIGHT END *********************************/


/************************************ DOCK LEFT **********************************/
/**
 * dock right - 左摆行为
 * NOTE:
 * 触发条件: 当中间接收头收到F4信号时，触发
 * 退出条件: 无
 */
DOCK_FN_DECL(docking_left)
{
	TransVel  left_vel;
	TransVel  right_vel;
	BOOLEAN already_mid = FALSE;
	S16 left_start_angle = 0;
    S16 angle = 0;
	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_left\r\n");
while(1)
{
	if (already_mid == FALSE)
	{
		if (recently_near_dock.current_state)
		{
			left_vel = DOCKING_NEAR_SLOWEST_SPEED;
			right_vel = DOCKING_NEAR_SLOW_SPEED;
		}
		else
		{
			left_vel = DOCKING_SLOWEST_SPEED;
			right_vel = DOCKING_SLOW_SPEED;
		}
	}
	else
	{
		left_vel = 0;
		right_vel = 0;
	}

	set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
}
 

	return ;
}

BOOLEAN docking_left_run_when(void)
{
	if(!docking_dock_where_abort)
		return FALSE;

    if(recently_docking_left.current_state&&\
		current_dock_behavior()!=DOCKING_RIGHT)
        return TRUE;
	else
		return FALSE;
}

void dock_left_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_LEFT;
	dock_funtion.start_when = NULL;
	dock_funtion.run_when = &docking_left_run_when;
	dock_funtion.abort_when = NULL;
	dock_funtion.abort_code = NULL;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_left;

	register_dock_function(&dock_funtion);

	return;
}
/********************************** DOCK LEFT END *********************************/


/********************************* DOCK GO FORWARD ********************************/
/**
 * dock go forward - 直行行为
 * NOTE:
 * 触发条件: 当中间接收头收到中间信号时，触发
 * 退出条件: 无
 */
DOCK_FN_DECL(docking_go_forward)
{
	TransVel  left_vel;
	TransVel  right_vel;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_go_forward\r\n");
	if (recently_near_dock.current_state)
	{
		left_vel = FORWARD_NEAR_SLOW_SPEED;
		right_vel = FORWARD_NEAR_SLOW_SPEED;
	}
	else
	{
		left_vel = FORWARD_SLOW_SPEED;
		right_vel = FORWARD_SLOW_SPEED;
	}

	set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
	return ;
}

BOOLEAN docking_go_forward_run_when(void)
{
	if(!docking_dock_where_abort)
		return FALSE;
		if ((recently_docking_go_forward.current_state||\
		current_dock_behavior()==DOCKING_GO_FORWARD)&&\
		((recently_docking_left.current_state&&\
		recently_docking_right.current_state)||\
		(!recently_docking_left.current_state&&\
		!recently_docking_right.current_state)))
		return TRUE;
	else
		return FALSE;
}

void docking_go_forward_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_GO_FORWARD;
	dock_funtion.start_when = NULL;
	dock_funtion.run_when = &docking_go_forward_run_when;
	dock_funtion.abort_when = NULL;
	dock_funtion.abort_code = NULL;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_go_forward;

	register_dock_function(&dock_funtion);

	return;
}
/******************************* DOCK GO FORWARD END ******************************/


/********************************** DOCK CORRECT **********************************/
/**
 * dock correct - 矫正左右摆行为
 * NOTE:
 * 触发条件: 当左右摆行为过程中，中间接收头丢失信号时，触发
 * 退出条件: 无
 */
static AM_LeftRight docking_correct_direction = AM_RIGHT;
static S64 correct_target_heading = 0;
static BOOLEAN docking_correct_abort = FALSE;
void set_docking_correct_abort(void)
{
	docking_correct_abort = TRUE;

	return;
}

void docking_correct_abort_code(void)
{

	docking_correct_abort = FALSE;

	return;
}

BOOLEAN docking_correct_abort_when(void)
{

	if (docking_correct_abort)
		return TRUE;
	else
		return FALSE;
}

DOCK_FN_DECL(docking_correct)
{
	TransVel  left_vel;
	TransVel  right_vel;
	BOOLEAN already_mid = FALSE;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_correct\r\n");

	if (already_mid == FALSE)
	{
		if (docking_correct_direction == AM_LEFT)
		{
			if (recently_near_dock.current_state)
			{
				left_vel = DOCKING_NEAR_SLOWEST_SPEED;
				right_vel = DOCKING_NEAR_SLOW_SPEED;
			}
			else
			{
				left_vel = DOCKING_SLOWEST_SPEED;
				right_vel = DOCKING_SLOW_SPEED;
			}
		}
		else if (docking_correct_direction == AM_RIGHT)
		{
			if (recently_near_dock.current_state)
			{
				left_vel = DOCKING_NEAR_SLOW_SPEED;
				right_vel = DOCKING_NEAR_SLOWEST_SPEED;
			}
			else
			{
				left_vel = DOCKING_SLOW_SPEED;
				right_vel = DOCKING_SLOWEST_SPEED;
			}
		}
	}
	else
	{
		left_vel = 0;
		right_vel = 0;
	}

	set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);

	if ((abs(correct_target_heading - get_gyro_angle())) > 30)
		set_docking_correct_abort();

	return ;
}

BOOLEAN docking_correct_start_when(void)
{
	if ((current_dock_behavior() == DOCKING_LEFT) && \
		(docking_left_run_when() == FALSE))
	{
		docking_correct_direction = AM_RIGHT;
		correct_target_heading = get_gyro_angle();
		return TRUE;
	}
	else if ((current_dock_behavior() == DOCKING_RIGHT) && \
		(docking_right_run_when() == FALSE))
	{
		docking_correct_direction = AM_LEFT;
		correct_target_heading = get_gyro_angle();
		return TRUE;
	}
	else
		return FALSE;
}

void dock_correct_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_CORRECT;
	dock_funtion.start_when = &docking_correct_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_correct_abort_when;
	dock_funtion.abort_code = &docking_correct_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_correct;

	register_dock_function(&dock_funtion);

	return;
}
/******************************** DOCK CORRECT END ********************************/

/************************************ DOCK WHERE **********************************/
/**
* dock where - 寻找座子方向
* NOTE:
* 触发条件:
* 退出条件:
*/
static BOOLEAN docking_where_abort = FALSE;

typedef struct {
 BOOLEAN first_check;
 S16 angle_range[5][2];
 U8 angle_range_it;
 S16 start_angle;
 S16 last_angle;
} check_dock_dir_info;

S16 angle_differ(S16 angle1,S16 angle2)
{
  S16 diff = abs(angle1-angle2);
  diff = diff>180?360-diff:diff;
  return diff;
}

void reset_check_dock_dir_info(check_dock_dir_info * _cdd)
{
  _cdd->first_check = TRUE;
  _cdd->start_angle = get_gyro_angle(); 
  _cdd->angle_range_it = 0;
  _cdd->last_angle = 0;
  memset(_cdd->angle_range,0,sizeof(_cdd->angle_range));
  return;
}

BOOLEAN check_continue_signal(check_dock_dir_info * _cdd)
{
  if(_cdd->angle_range_it>=5)
    return TRUE;
  if(check_recently_all_mid())
  {
    S16 current_angle=get_gyro_angle();
    if(_cdd->first_check)
    {
      _cdd->angle_range[0][0] =  current_angle;
      _cdd->first_check = FALSE;
      _cdd->last_angle = current_angle;
    }
    else 
    {
      if(current_angle-_cdd->last_angle>10)
      {
        _cdd->angle_range[_cdd->angle_range_it][1] = _cdd->last_angle;
        _cdd->angle_range_it++;
        if(_cdd->angle_range_it<5)
          _cdd->angle_range[_cdd->angle_range_it][0] = current_angle;
        else return TRUE;
      }
      _cdd->last_angle = current_angle;
    }
}
return TRUE;
}
void mrege_signal(check_dock_dir_info * _cdd)
{
  _cdd->angle_range[_cdd->angle_range_it][1] = _cdd->last_angle;
  if(_cdd->angle_range_it>0)
  {
    S16 end_angle = _cdd->angle_range[_cdd->angle_range_it][1];
    S16 start_angle =  _cdd->angle_range[0][0];
    if(abs(end_angle-360-start_angle)<15)
    {
      _cdd->angle_range[0][0]=_cdd->angle_range[_cdd->angle_range_it][0];
      _cdd->angle_range_it--;
    }
  }
}
S16 cacl_dock_turn_angle(check_dock_dir_info * _cdd)
{
  S16 min_angle = 0;
  S16 max_angle = 0;
  S16 last_range_angle = 0;
  S16 angle = 0;
  mrege_signal(_cdd);
  for(int i= 0; i<=_cdd->angle_range_it;i++)
  {
    printf("dock siganl:%d(%d,%d)\r\n",i,_cdd->angle_range[i][0],_cdd->angle_range[i][1]);
    S16 current_range_angle = angle_differ(_cdd->angle_range[i][0],_cdd->angle_range[i][1]);
    if(current_range_angle>last_range_angle)
    {
      min_angle = _cdd->angle_range[i][0];
      max_angle = _cdd->angle_range[i][1];
      last_range_angle = current_range_angle;
    }	
  }
  angle = abs(min_angle-max_angle)<180?
    (min_angle+max_angle)/2-get_gyro_angle():
    (min_angle+max_angle)/2-get_gyro_angle()-180;
    angle = abs(angle)<180?angle:(angle>0?angle-360:angle+360);
    return angle;
}

BOOLEAN check_dock_dir(S16 *min_angle,S16 *max_angle,BOOLEAN *first_check)
{
  S16 gyro_angle;
  S16 range_angle;
  
  if(check_recently_all_mid())
  {
    gyro_angle = get_gyro_angle();
    
    if(*first_check)
    {
      *first_check = FALSE;
      *min_angle = gyro_angle;
      *max_angle = *min_angle;
    }
    else
    {
      S16 dst_min_angle = *min_angle;
      S16 dst_max_angle = *max_angle;
      range_angle =angle_differ(dst_min_angle,dst_max_angle);
      if(angle_differ(gyro_angle,dst_max_angle)>range_angle)
      {
        *min_angle = gyro_angle;
        *max_angle = dst_max_angle;
        range_angle = angle_differ(gyro_angle,dst_max_angle);
      }
      if(angle_differ(dst_min_angle,gyro_angle)>range_angle)
      {
        *min_angle = dst_min_angle;
        *max_angle = gyro_angle;
        range_angle = angle_differ(dst_min_angle,gyro_angle);
      }
    }
  //printf("angle:%d,%d(in:%d)!!!",*min_angle,*max_angle,gyro_angle);
}
return TRUE;
}
BOOLEAN check_dock_condition(U8 *dock_condition)
{
  if(recently_docking_go_forward.current_state||\
    recently_docking_left.current_state||\
      recently_docking_right.current_state)
    *dock_condition|= DOCK_WHERE_HAS_CONDITION;
  if(recently_all_ir_center.current_state)
    *dock_condition|= DOCK_WHERE_HAS_CENTER;
  return TRUE;
}

DOCK_FN_DECL(docking_dock_where)
{
#if 0
  S8 result = 0;
  S16 min_angle;
  S16 max_angle;
  BOOLEAN first_check = TRUE;
  U8 dock_condition = 0;
  dprintf(DEBUG_DOCK_BEHAVIOR, "docking_where\r\n");
  
  clear_specify_debouncer(&recently_all_ir_signal);
  docking_dock_where_abort = FALSE;
  
  AM_GO_TO_PLACE(360, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                 (check_dock_dir(&min_angle,&max_angle,&first_check)&&\
                   check_dock_condition(&dock_condition)),CARE_CLIFF,result);
  if(!first_check&&dock_condition)
  {
    S16 angle = 0;
    angle = abs(min_angle-max_angle)<180?
      (min_angle+max_angle)/2-get_gyro_angle():
      (min_angle+max_angle)/2-get_gyro_angle()-180;
      angle = abs(angle)<180?angle:(angle>0?angle-360:angle+360);
      am_go_to_place(angle, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                     TRUE,CARE_CLIFF,&result);
      docking_dock_where_abort = TRUE;
      printf("angle:%d(%d,%d)!!!!\r\n",angle,min_angle,max_angle);
      am_go_to_place(-40, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                     TRUE,CARE_CLIFF,&result);
      am_go_to_place(80, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                     TRUE,CARE_CLIFF,&result);
      am_go_to_place(-40, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                     TRUE,CARE_CLIFF,&result);
      am_go_to_place(360, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                     TRUE,CARE_CLIFF,&result);
      printf("[dock_error]:dock_where error1\r\n");
  }
  else
  {
    if(recently_all_ir_signal.current_state)
    {
      docking_state.dock_failured=DOCKING_FAIL_NO_CONDITION;
      printf("[dock_failure]dock_where has signal\r\n");
    }
    else
    {
      //docking_where_abort = TRUE;
      docking_state.dock_failured=DOCKING_FAIL_NO_SIGNAL;
      printf("[dock_failure]dock_where has not signal\r\n");
    }
  }
	return;
#else
  S8 result = 0;
  S16 angle = 0;
  U8 dock_condition = 0;
  BOOLEAN dock_condition_result = 0;
  check_dock_dir_info _check_dock_dir;
  dprintf(DEBUG_DOCK_BEHAVIOR, "docking_where\r\n");
  
  clear_specify_debouncer(&recently_all_ir_signal);
  reset_check_dock_dir_info(&_check_dock_dir);
  docking_dock_where_abort = FALSE;
  
  AM_GO_TO_PLACE(360, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                 (check_continue_signal(&_check_dock_dir)&&\
                   check_dock_condition(&dock_condition)),CARE_CLIFF,result);
  if(docking_state.dock_start_flag)
  {
     dock_condition_result = dock_condition&DOCK_WHERE_HAS_CONDITION;
     docking_state.dock_start_flag = FALSE;
  }
  else 
  {
    dock_condition_result = (dock_condition&DOCK_WHERE_HAS_CONDITION)&&\
         (dock_condition&DOCK_WHERE_HAS_CENTER);
  }
  if(_check_dock_dir.first_check&&dock_condition_result)
  {
    docking_dock_where_abort = TRUE;
    am_go_to_place(360, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                   TRUE,CARE_CLIFF,&result);
    printf("[dock_error]:no all mid\r\n");
  }	
  else if(!_check_dock_dir.first_check&&dock_condition_result)
  {
    angle= cacl_dock_turn_angle(&_check_dock_dir);
    am_go_to_place(angle, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                   TRUE,CARE_CLIFF,&result);
    docking_dock_where_abort = TRUE;
    printf("angle:%d!!!!\r\n",angle);
    am_go_to_place(-40, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                   TRUE,CARE_CLIFF,&result);
    am_go_to_place(80, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                   TRUE,CARE_CLIFF,&result);
    am_go_to_place(-40, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                   TRUE,CARE_CLIFF,&result);
    am_go_to_place(360, DOCKING_NEAR_SLOW_SPEED,DOCKING_NEAR_SLOW_SPEED,
                   TRUE,CARE_CLIFF,&result);
    printf("[dock_error]:dock_where error1\r\n");
  }
  else
  {
    if(recently_all_ir_signal.current_state)
    {
      docking_state.dock_failured=DOCKING_FAIL_NO_CONDITION;
      printf("[dock_fail]dock_where has signal\r\n");
    }
    else
    {
      //docking_where_abort = TRUE;
      docking_state.dock_failured=DOCKING_FAIL_NO_SIGNAL;
      printf("[dock_fail]dock_where has not signal\r\n");
    }
  }
  return;
#endif
}

BOOLEAN docking_where_run_when(void)
{		
		return TRUE;
}

void docking_where_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_WHERE;
	dock_funtion.start_when = NULL;
	dock_funtion.run_when = &docking_where_run_when;
	dock_funtion.abort_when = NULL;
	dock_funtion.abort_code = NULL;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_dock_where;
	register_dock_function(&dock_funtion);

	return;
}
/******************************* DOCK WHERE END ******************************/

extern void setup_ir_config();
extern void setup_dock_api();
dock_config_t* dock_new_init(void)
{
	dock_success_register();
	docking_bounce_register();
	dock_right_register();
	dock_left_register();
	docking_go_forward_register();
	dock_correct_register();
	docking_avoid_obstacle_register(); 
    docking_where_register();

    for (int i = 0; i < IR_LOCAL_MAX; ++i)
        register_debouncer(&recently_receiver[i]);
    for (int i = 0; i < IR_LOCAL_MAX; ++i)
        register_debouncer(&recently_avoid_receiver[i]);
    for (int i = 0; i < 3; ++i)
        register_debouncer(&recently_sender[i]);
    register_debouncer(&recently_signal_short_term);
    register_debouncer(&recently_near_dock_2);
    register_debouncer(&recently_center);
    register_debouncer(&recently_facing_dock);

	register_debouncer(&recently_signal);
	register_debouncer(&recently_near_dock);
	register_debouncer(&recently_docking_left);
	register_debouncer(&recently_docking_right);
	register_debouncer(&recently_docking_go_forward);
	register_debouncer(&recently_docking_go_forward_right);
    register_debouncer(&recently_docking_go_forward_left);
    register_debouncer(&recently_docking_go_forward_onlyright);
    register_debouncer(&recently_docking_go_forward_onlyleft);
	register_debouncer(&recently_check_recently_all_mid);
	//register_debouncer(&recently_bump);
	register_debouncer(&recently_all_ir_center);
	register_debouncer(&recently_all_ir_signal);
	for (int i = 0; i < IR_LOCAL_MAX; ++i)
        register_debouncer(&recently_left_receiver[i]);
	for (int i = 0; i < IR_LOCAL_MAX; ++i)
        register_debouncer(&recently_right_receiver[i]);

	register_dock_signals(&robot_get_dock_signals);
	//register_random_conut(&dock_get_random_count);

    setup_ir_config();
    setup_dock_api();

	dock_config.max_ir_chan = IR_MAX_RECV;

	/* 圆泡看到圆泡信号 */
	dock_config.dock_avoid_chan = 0;
	dock_config.dock_avoid_chan = ((0x1<<IR_LOCAL_MID_LEFT)|(0x1<<IR_LOCAL_MID_RIGHT)|(0x1<<2)|(0x1<<3));
	/* 双目看到圆泡信号 */
	dock_config.binocular_see_avoid_chan = 0;
	dock_config.binocular_see_avoid_chan = ((0x1<<IR_LOCAL_MID_LEFT)|(0x1<<IR_LOCAL_MID_RIGHT));

	dock_config.aovw_chan = 0;
	dock_config.aovw_chan = ((0x1<<IR_LOCAL_MID_LEFT)|(0x1<<IR_LOCAL_MID_RIGHT)/*|\
									(0x1<<IR_LOCAL_LEFT)|(0x1<<IR_LOCAL_RIGHT)*/);
	dock_config.dock_signals_type.dock_closed = DOCK_CLOSE_BEACON;
	dock_config.dock_signals_type.left_signal = LEFT_BEACON_BYTE;
	dock_config.dock_signals_type.right_signal = RIGHT_BEACON_BYTE;
	dock_config.dock_signals_type.center_signal = BOTH_BEACONS_BYTE;
	dock_config.dock_signals_type.RESERVE1 = 0xff;
	dock_config.dock_signals_type.RESERVE2 = 0xff;
	dock_config.dock_signals_type.aovw_signal = AOVW_BYTE;

	dock_config.success_behavior_id = DOCKING_SUCCESS;
	dock_config.first_behavior_id = DOCKING_WHERE;

#ifdef IR_WIRELESS
    InitARF2496k();
    PartnershipRF();
#endif

	return &dock_config;
}

void dock_new_start(void)
{
	set_lighttouch_enable(0);
	turn_on_touch_bump();
	clear_debouncer();
	robot_sidebrush_vols_set(SIDE_BRUSH_MAX_VOLTAGE/2); 
	robot_pos_get(&dock_start_pose);
	docking_parameter_init();
    set_dock_leave_dockagain(0);
	dock_core_enable();
	return;
}

BOOLEAN dock_new_end(U8 *uTerm)
{
    if(recently_near_dock.current_state&&!docking_state.near_dock)
	{
	    set_near_dock_context(TRUE);
	    docking_state.near_dock = TRUE;
        set_line_dock_trigger_off(NEAR_LINE_DOCK_TRIGGER_OFF);
	}
    else if(!recently_near_dock.current_state&&docking_state.near_dock)
    {
        set_near_dock_context(FALSE);
	    docking_state.near_dock = FALSE;
        set_line_dock_trigger_off(LINE_DOCK_TRIGGER_OFF);
    }
    
    //printf("###random_behavior_count=%d\r\n",docking_state.random_behavior_count);
	if ((docking_state.dock_finished == FALSE) && \
		(docking_state.random_behavior_count < DOCKINT_RANDOM_THRESHOLD)&&\
		docking_state.dock_failure_count<DOCKING_MAX_FAILURE_COUNT&&\
		docking_state.avoid_obstacle_count<DOCKING_MAX_AVOID_OBSTACLE_COUNT&&\
		!docking_state.dock_failured)
		return FALSE;

	if (docking_state.dock_finished == TRUE)
		*uTerm = DOCKING_SUCESS;
	else if(docking_state.dock_failured)
		*uTerm = docking_state.dock_failured;
	else if (docking_state.random_behavior_count >= DOCKINT_RANDOM_THRESHOLD||\
		docking_state.dock_failure_count>=DOCKING_MAX_FAILURE_COUNT)
		*uTerm = DOCKING_FAIL_ATTEMPT_MAX;
	else if(docking_state.avoid_obstacle_count>=DOCKING_MAX_AVOID_OBSTACLE_COUNT)
		*uTerm = DOCKING_FAIL_OBSTACLE;

	dock_core_disable();
	clear_debouncer();
	set_near_dock_context(FALSE);
	if(!docking_state.dock_finished) 
		robot_sidebrush_vols_set(SIDE_BRUSH_MAX_VOLTAGE/2); 
	return TRUE;
}

void set_dock_new_end(void)
{
  if (dock_is_enable())
  {
    docking_state.dock_finished = TRUE;
    dock_core_disable();
  }
}

#endif // AM_DOCKING_METHOD