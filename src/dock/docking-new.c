//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "docking-new.h"
#include "docking-sensors.h"
#include "sensor/sensor.h"
#ifdef IR_WIRELESS
#include "dock_ir_signal.h"
#include "wireless/arf2496k.h"
#endif
#include "monitor/robot_batter.h"
#include "motor/robot_brush.h"
#ifndef AM_DOCKING_METHOD
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

#define DOCKING_TRUN_SLOWEST_SPEED    (120)
#define DOCKING_TRUN_SLOW_SPEED       (240)
#define DOCKING_NEAR_SLOWEST_SPEED    (70)
#define DOCKING_NEAR_SLOW_SPEED       (100)
#define DOCKING_SLOWEST_SPEED         (110)
#define DOCKING_SLOW_SPEED            (150)
#define FORWARD_NEAR_SLOW_SPEED       (90)
#define FORWARD_SLOW_SPEED            (180)
#define FORWARDSPEED                  (240)

#define VERIFY_HOLD_CNT               (2)

#define CARE_BUMP                     (1)
#define CARE_CLIFF                    (2)
#define SIDE_BRUSH_SPEED              (550)

static DockingState docking_state =
{
	0,
	FALSE,
	0,
	0,
	0x0FFF,
};

static dock_config_t dock_config;

BOOLEAN docking_left_right_start_when(void);
BOOLEAN docking_left_run_when(void);
BOOLEAN docking_right_run_when(void);
void docking_find_buoy_start_set(void);
void docking_find_set_orientation_direction(void);
void docking_find_set_head_direction(void);
BOOLEAN docking_ahead_start_when(void);

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
	dock_config.dock_again = state;
	return;
}

dock_config_t *get_dock_config(void)
{
	return &dock_config;
}

static void docking_parameter_init(void)
{
	docking_state.dock_finished = FALSE;
	docking_state.random_behavior_count = 0;
	docking_state.random_current_count = 0;
	docking_state.state_cnt = 0;
	docking_state.dock_angle = 0x0FFF;
#ifdef IR_WIRELESS
	docking_state.wireless_data = 0;
#endif
	return;
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
	robot_sidebrush_vols_set(1); 
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
		DRIVE_GO(-200,FORWARDSPEED,(!charging_detect()),0,result);
                if(charging_detect())
                    docking_state.dock_finished = TRUE;
                else
		    set_docking_success_abort();
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

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_bounce\r\n");

	while((get_bump_state() !=0) || (get_cliff_state() != 0)){};

	DRIVE_GO(-50,FORWARDSPEED,TRUE,(CARE_CLIFF),result);

	AM_GO_TO_PLACE(180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

	DRIVE_GO(600,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,\
		(!docking_ahead_start_when()), \
		CARE_CLIFF,result);

	set_docking_bounce_abort();

	return ;
}

BOOLEAN docking_bounce_start_when(void)
{
	if (((get_bump_state() !=0) || (get_cliff_state() != 0)) && \
		(((recently_near_dock.current_state) && \
		((recently_center_left_focus.current_state && recently_center_right_focus.current_state)) && \
		(current_dock_behavior() != DOCKING_FORCE_FIELD) && \
		(current_dock_behavior() != DOCKING_FIND_BUOY)) || \
		(current_dock_behavior() == DOCKING_SUCCESS)))
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

	AM_GO_TO_PLACE(docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

	DRIVE_GO(100,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(-docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

	DRIVE_GO(150,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(-docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

	DRIVE_GO(100,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(docking_avoid_obstacle_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

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



/********************************* DOCK LINE BOUNCE *******************************/
/**
 * dock bounce - 没有正对着充电座时的碰撞处理
 * NOTE:此行为与dock line行为配合，实现随机功能
 * 触发条件: 没有正对着充电座时，发生bump或cliff
 * 退出条件: 无
 */
static BOOLEAN docking_line_bounce_abort = FALSE;
static S16 docking_line_bounce_angle = 0;
void set_docking_line_bounce_abort(void)
{
	docking_line_bounce_abort = TRUE;
	return;
}

BOOLEAN docking_line_bounce_abort_when(void)
{

	if(docking_line_bounce_abort != FALSE)
		return TRUE;
	else
		return FALSE;
}

void docking_line_bounce_abort_code(void)
{
	docking_line_bounce_abort = FALSE;
	return;
}

DOCK_FN_DECL(docking_line_bounce)
{
	S8 result = 0;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_line_bounce\r\n");

	while((get_cliff_state() != 0) || (get_bump_state() != 0)){};

	AM_GO_TO_PLACE(docking_line_bounce_angle,DOCKING_TRUN_SLOWEST_SPEED,\
		DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

	set_docking_line_bounce_abort();

	return ;
}

BOOLEAN docking_line_bounce_start_when(void)
{
	BumpState bumped_state = get_bump_state();
	CliffState cliffed_state = get_cliff_state();
	U16 angle = 0;

	if (current_dock_behavior() == DOCKING_LINE_BOUNCE)
		return FALSE;

	if (((bumped_state !=0) || (cliffed_state != 0)) && \
		(!docking_bounce_start_when()))
	{
		angle = get_random();

		if (bumped_state & BUMP_FRONT_LEFT)
		{
			docking_line_bounce_angle = -angle;
		}
		else if(bumped_state & BUMP_FRONT_RIGHT)
		{
			docking_line_bounce_angle = angle;
		}
		else if(bumped_state & BUMP_FRONT_CENTER)
		{
			if (docking_line_bounce_angle > 0)
			{
				docking_line_bounce_angle = angle;
			}
			else
			{
				docking_line_bounce_angle = -angle;
			}
		}
		else if ((cliffed_state & CLIFF_FRONT_LEFT) || (cliffed_state & CLIFF_FRONT_RIGHT))
		{
			if (docking_line_bounce_angle > 0)
			{
				docking_line_bounce_angle = angle;
			}
			else
			{
				docking_line_bounce_angle = -angle;
			}
		}
		else if (cliffed_state & CLIFF_SIDE_LEFT)
		{
			docking_line_bounce_angle = -angle;
		}
		else if (cliffed_state & CLIFF_SIDE_RIGHT)
		{
			docking_line_bounce_angle = angle;
		}

		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void docking_line_bounce_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_LINE_BOUNCE;
	dock_funtion.start_when = &docking_line_bounce_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_line_bounce_abort_when;
	dock_funtion.abort_code = &docking_line_bounce_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_line_bounce;

	register_dock_function(&dock_funtion);

	return;
}
/******************************* DOCK LINE BOUNCE END *****************************/


/************************************ DOCK AHEAD **********************************/
/**
 * dock ahead - 样机位于座子正前方
 * NOTE:
 * 触发条件:
 * 退出条件:
 */
static BOOLEAN docking_dock_ahead_abort = FALSE;
static AM_LeftRight dock_ahead_dir = AM_RIGHT;
void set_docking_ahead_abort(void)
{
	docking_dock_ahead_abort = TRUE;

	return;
}

BOOLEAN docking_ahead_abort_when(void)
{
	if ((docking_dock_ahead_abort != FALSE) || \
		(recently_center_left_focus.current_state && \
		recently_center_right_focus.current_state))
		return TRUE;
	else
		return FALSE;
}

void docking_ahead_abort_code(void)
{

	docking_dock_ahead_abort = FALSE;

	return;
}

DOCK_FN_DECL(docking_dock_ahead)
{
	S8 result = 0;
	S16 angle = 0;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_dock_ahead\r\n");

	if (dock_ahead_dir == AM_LEFT)
	{
		dprintf(DEBUG_DOCK_BEHAVIOR, "left\r\n");

		angle = 90;
	}
	else if (dock_ahead_dir == AM_RIGHT)
	{
		dprintf(DEBUG_DOCK_BEHAVIOR, "right\r\n");

		angle = -90;
	}


	AM_GO_TO_PLACE(angle, DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,\
		(CARE_CLIFF|CARE_BUMP),result);

	set_docking_ahead_abort();

	return;
}

BOOLEAN docking_ahead_start_when(void)
{
	if (current_dock_behavior() == DOCKING_DOCK_AHEAD)
		return FALSE;

	if (recently_center_left_focus.current_state || \
		recently_center_right_focus.current_state)
		return FALSE;

	if (((recently_right_right.current_state && recently_left_backright.current_state) || \
		(recently_left_right.current_state && recently_right_backright.current_state))&& \
		(current_dock_behavior() != DOCKING_LEFT) && \
		(current_dock_behavior() != DOCKING_RIGHT) && \
		(current_dock_behavior() != DOCKING_GO_FORWARD) && \
		(current_dock_behavior() != DOCKING_CORRECT) &&\
		(!(recently_center_left_focus.current_state && \
		recently_center_right_focus.current_state)))
	{
		dock_ahead_dir = AM_RIGHT;
		return TRUE;
	}
	else if (((recently_left_left.current_state && recently_right_backleft.current_state) ||
		(recently_right_left.current_state && recently_left_backleft.current_state))&& \
		(current_dock_behavior() != DOCKING_LEFT) && \
		(current_dock_behavior() != DOCKING_RIGHT) && \
		(current_dock_behavior() != DOCKING_GO_FORWARD) && \
		(current_dock_behavior() != DOCKING_CORRECT) &&\
		(!(recently_center_left_focus.current_state && \
		recently_center_right_focus.current_state)))
	{

		dock_ahead_dir = AM_LEFT;
		return TRUE;
	}
	else
		return FALSE;
}

void docking_ahead_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_DOCK_AHEAD;
	dock_funtion.start_when = &docking_ahead_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_ahead_abort_when;
	dock_funtion.abort_code = &docking_ahead_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_dock_ahead;

	register_dock_function(&dock_funtion);

	return;
}
/******************************* DOCK LEFT RIGHT END ******************************/


/******************************** DOCK LEFT RIGHT *********************************/
/**
 * dock left right - 收到F4或F8时，找中线行为
 * NOTE:
 * 触发条件: 样机收到F4或F8信号
 * 退出条件: 样机中间接收头收到F2信号，或同时收到F4和F8信号
 */
static BOOLEAN docking_left_right_abort = FALSE;
static S16 docking_left_right_left = 0;
static S16 docking_left_right_right = 0;
static AM_LeftRight left_right_dir = AM_RIGHT;
void set_docking_left_right_abort(void)
{
	U32 turn_stop_time = 0;

	turn_stop_time = timer_ms();
	do
	{
		set_motor_vels(0, 0, ACCELERATION_MAX);
	}
	while (timer_elapsed(turn_stop_time) < 100);

	docking_left_right_abort = TRUE;

	return;
}

BOOLEAN docking_left_right_abort_when(void)
{
	if ((docking_left_right_abort != FALSE) || \
		(!recently_signal.current_state) || \
		((recently_center_left_focus.current_state && \
		recently_center_right_focus.current_state) && \
		(recently_docking_go_forward_right.current_state || \
		recently_docking_go_forward_left.current_state)) || \
		(recently_left_left.current_state && recently_right_right.current_state))
		return TRUE;
	else
		return FALSE;
}

void docking_left_right_abort_code(void)
{

	docking_left_right_abort = FALSE;

	return;
}

DOCK_FN_DECL(docking_left_right)
{
	S8 result = 0;
	S16 angle = 0;
	BOOLEAN quit_this_behavior = FALSE;
	S8 go_count = 0;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_left_right\r\n");

	if (left_right_dir == AM_LEFT)
	{
		dprintf(DEBUG_DOCK_BEHAVIOR, "left\r\n");

		angle = 120;
	}
	else if (left_right_dir == AM_RIGHT)
	{
		dprintf(DEBUG_DOCK_BEHAVIOR, "right\r\n");

		angle = -120;
	}

DOCKING_LEFT_RIGHT_STEP:

	if (go_count > 6)
	{
		AM_GO_TO_PLACE(180, docking_left_right_left,docking_left_right_right,\
		TRUE,\
		(CARE_CLIFF|CARE_BUMP),result);
		set_docking_left_right_abort();
	}
	else
		go_count++;

	AM_GO_TO_PLACE(angle, docking_left_right_left,docking_left_right_right,\
		(!docking_left_right_abort_when()),\
		(CARE_CLIFF|CARE_BUMP),result);

	if (result != 0)
	{
		docking_line_bounce_start_when();

		AM_GO_TO_PLACE(docking_line_bounce_angle,DOCKING_TRUN_SLOWEST_SPEED,\
			DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

		set_docking_left_right_abort();
		return;
	}

	if (left_right_dir == AM_LEFT)
	{
		if (recently_right_left.current_state)
		{
			quit_this_behavior = TRUE;
		}
	}
	else
	{
		if (recently_left_right.current_state)
		{
			quit_this_behavior = TRUE;
		}
	}

	AM_GO_TO_PLACE(-angle, docking_left_right_left,docking_left_right_right, \
		(!((recently_left_left.current_state && recently_left_backleft.current_state) || \
		(recently_right_right.current_state && recently_right_backright.current_state) || \
		(docking_left_right_abort_when()))), \
		(CARE_CLIFF|CARE_BUMP),result);

	if (result != 0)
	{
		docking_line_bounce_start_when();

		AM_GO_TO_PLACE(docking_line_bounce_angle,DOCKING_TRUN_SLOWEST_SPEED,\
			DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

		set_docking_left_right_abort();
		return;
	}

	if (left_right_dir == AM_LEFT)
	{
		if (recently_right_left.current_state)
		{
			quit_this_behavior = TRUE;
		}
	}
	else
	{
		if (recently_left_right.current_state)
		{
			quit_this_behavior = TRUE;
		}
	}

	if (left_right_dir == AM_LEFT)
	{
		if (recently_right_left.current_state)
		{
			DRIVE_GO(20,FORWARDSPEED,TRUE, \
				(CARE_CLIFF|CARE_BUMP),result);
			quit_this_behavior = TRUE;
		}
		else
		{
			DRIVE_GO(100,FORWARDSPEED,(!recently_right_left.current_state), \
				(CARE_CLIFF|CARE_BUMP),result);
		}
	}
	else
	{
		if (recently_left_right.current_state)
		{
			DRIVE_GO(20,FORWARDSPEED,TRUE, \
				(CARE_CLIFF|CARE_BUMP),result);
			quit_this_behavior = TRUE;
		}
		else
		{
			DRIVE_GO(100,FORWARDSPEED,(!recently_left_right.current_state), \
				(CARE_CLIFF|CARE_BUMP),result);
		}
	}

	if (result != 0)
	{
		if (angle > 0)
		{
			AM_GO_TO_PLACE(90,DOCKING_TRUN_SLOWEST_SPEED,\
				DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);
		}
		else
		{
			AM_GO_TO_PLACE(-90,DOCKING_TRUN_SLOWEST_SPEED,\
				DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);
		}

		DRIVE_GO(150,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);
		if (result != 0)
		{
			docking_line_bounce_start_when();

			AM_GO_TO_PLACE(docking_line_bounce_angle,DOCKING_TRUN_SLOWEST_SPEED,\
			DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);

			set_docking_left_right_abort();
			return;
		}

		if (angle > 0)
		{
			AM_GO_TO_PLACE(-90,DOCKING_TRUN_SLOWEST_SPEED,\
				DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);
		}
		else
		{
			AM_GO_TO_PLACE(90,DOCKING_TRUN_SLOWEST_SPEED,\
				DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);
		}

		DRIVE_GO(50,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);
	}

	if (recently_right_left.current_state || recently_left_right.current_state || quit_this_behavior)
	{
		if (left_right_dir == AM_LEFT)
		{
			AM_GO_TO_PLACE(90, docking_left_right_left,docking_left_right_right,\
				(!(docking_left_right_abort_when())),\
				(CARE_CLIFF|CARE_BUMP),result);
		}
		else
		{
			AM_GO_TO_PLACE(-90, docking_left_right_left,docking_left_right_right,\
				(!(docking_left_right_abort_when())),\
				(CARE_CLIFF|CARE_BUMP),result);
		}

		if (result != 0)
		{
			docking_line_bounce_start_when();

			AM_GO_TO_PLACE(docking_line_bounce_angle,DOCKING_TRUN_SLOWEST_SPEED,\
						DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);
		}

		set_docking_left_right_abort();

		return;
	}

	goto DOCKING_LEFT_RIGHT_STEP;
}

BOOLEAN docking_left_right_start_when(void)
{
	if (current_dock_behavior() == DOCKING_LEFT_RIGHT)
		return FALSE;

	if (recently_center_left_focus.current_state || \
		recently_center_right_focus.current_state)
		return FALSE;

	if ((recently_right_right.current_state || recently_right_backright.current_state) && \
		(current_dock_behavior() != DOCKING_LEFT) && \
		(current_dock_behavior() != DOCKING_RIGHT) && \
		(current_dock_behavior() != DOCKING_GO_FORWARD) && \
		(current_dock_behavior() != DOCKING_DOCK_AHEAD) && \
		(current_dock_behavior() != DOCKING_CORRECT) &&\
		(!(recently_center_left_focus.current_state && \
		recently_center_right_focus.current_state)))
	{
		docking_left_right_left = DOCKING_TRUN_SLOWEST_SPEED;
		docking_left_right_right = DOCKING_TRUN_SLOWEST_SPEED;

		left_right_dir = AM_RIGHT;
		return TRUE;
	}
	else if ((recently_left_left.current_state || recently_left_backleft.current_state) && \
		(current_dock_behavior() != DOCKING_LEFT) && \
		(current_dock_behavior() != DOCKING_RIGHT) && \
		(current_dock_behavior() != DOCKING_GO_FORWARD) && \
		(current_dock_behavior() != DOCKING_DOCK_AHEAD) && \
		(current_dock_behavior() != DOCKING_CORRECT) &&\
		(!(recently_center_left_focus.current_state && \
		recently_center_right_focus.current_state)))
	{
		docking_left_right_left = DOCKING_TRUN_SLOWEST_SPEED;
		docking_left_right_right = DOCKING_TRUN_SLOWEST_SPEED;

		left_right_dir = AM_LEFT;
		return TRUE;
	}
	else
		return FALSE;
}

void docking_left_right_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_LEFT_RIGHT;
	dock_funtion.start_when = &docking_left_right_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_left_right_abort_when;
	dock_funtion.abort_code = &docking_left_right_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_left_right;

	register_dock_function(&dock_funtion);

	return;
}
/******************************* DOCK LEFT RIGHT END ******************************/


/********************************* DOCK FORCE FIELD *******************************/
/**
 * dock force field - 避开座子的行为，防止机器撞座
 * NOTE:实现绕座行为
 * 触发条件: 样机没有正对着充电座，但收到了圆泡信号
 * 退出条件: 无
 */
static BOOLEAN docking_force_field_abort = FALSE;
static S64 docking_force_field_orientation = 0;

void set_docking_force_field_abort(void)
{
	docking_force_field_abort = TRUE;
	return;
}

BOOLEAN docking_force_field_abort_when(void)
{

	if (docking_force_field_abort != FALSE)
		return TRUE;
	else
		return FALSE;
}

void docking_force_field_abort_code(void)
{
	docking_force_field_abort = FALSE;
	return;
}

U16 follow_force_field(AM_LeftRight follow_dir)
{
	TransVel  left_vel;
	TransVel  right_vel;
	S64 orientation = 0;
	S16 angle = 0;
	BOOLEAN already_90 = FALSE;

	if (follow_dir == AM_RIGHT)
	{
FOLLOW_FORCE_FIELD_RIGHT:
		orientation = get_gyro_angle();

		do
		{
			angle = orientation - get_gyro_angle();

			if (already_90 == FALSE)
			{
				if (abs(angle) >= 90)
					already_90 = TRUE;
			}

			if (abs(angle) > 180)
			{
				if (angle > 0)
				{
					angle = 360 - angle;
				}
				else
				{
					angle = 360 + angle;
				}
			}

			dprintf(DEBUG_DOCK_FORCE_FIELD, "right out\r\n");

			if (already_90)
			{
				if (recently_force_field_middle.current_state)
				{
					set_motor_vels(0, 0, ACCELERATION_MIN);
					return 0;
				}
				left_vel = 90;
				right_vel = 90;
				set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
			}
			else
			{
				left_vel = 20;
				right_vel = 90;
				set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
			}

			if (recently_bump.current_state)
			{
				docking_force_field_orientation = (get_gyro_angle()+180);
				return 0;
			}

			if (recently_left_right.current_state)
				return LEFT_BEACON_BYTE;

			if (recently_right_right.current_state)
				return RIGHT_BEACON_BYTE;

			if ((recently_center_left_focus.current_state && recently_center_right_focus.current_state))
				return (LEFT_BEACON_BYTE|RIGHT_BEACON_BYTE);

		} while(recently_follow_right_force_field.current_state);

		already_90 = FALSE;
		orientation = get_gyro_angle();

		do
		{
			angle = orientation - get_gyro_angle();
			if (abs(angle) > 350)
			{
				set_docking_force_field_abort();
				return 0;
			}
			if (abs(angle) > 180)
			{
				if (angle > 0)
				{
					angle = 360 - angle;
				}
				else
				{
					angle = 360 + angle;
				}
			}

			dprintf(DEBUG_DOCK_FORCE_FIELD, "right in\r\n");

			if ((abs(angle) >= 90) && (recently_force_field_middle.current_state))
			{
				set_motor_vels(0, 0, ACCELERATION_MIN);
				break;
			}
			else
			{
				left_vel = 90;
				right_vel = 20;
				set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
			}

			if (recently_bump.current_state)
			{
				docking_force_field_orientation = (get_gyro_angle()+180);
				return 0;
			}

			if (recently_left_right.current_state)
				return LEFT_BEACON_BYTE;

			if (recently_right_right.current_state)
				return RIGHT_BEACON_BYTE;

			if ((recently_center_left_focus.current_state && recently_center_right_focus.current_state))
				return (LEFT_BEACON_BYTE|RIGHT_BEACON_BYTE);

		} while(!recently_follow_right_force_field.current_state);

		goto FOLLOW_FORCE_FIELD_RIGHT;
	}
	else if (follow_dir == AM_LEFT)
	{
FOLLOW_FORCE_FIELD_LEFT:
		orientation = get_gyro_angle();

		do
		{
			angle = orientation - get_gyro_angle();

			if (already_90 == FALSE)
			{
				if (abs(angle) >= 90)
					already_90 = TRUE;
			}

			if (abs(angle) > 180)
			{
				if (angle > 0)
				{
					angle = 360 - angle;
				}
				else
				{
					angle = 360 + angle;
				}
			}

			dprintf(DEBUG_DOCK_FORCE_FIELD, "left out\r\n");

			if (already_90)
			{
				if (recently_force_field_middle.current_state)
				{
					set_motor_vels(0, 0, ACCELERATION_MIN);
					return 0;
				}
				left_vel = 90;
				right_vel = 90;
				set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
			}
			else
			{
				left_vel = 90;
				right_vel = 20;
				set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
			}

			if (recently_bump.current_state)
			{
				docking_force_field_orientation = (get_gyro_angle()+180);
				return 0;
			}

			if (recently_left_left.current_state)
				return LEFT_BEACON_BYTE;

			if (recently_right_left.current_state)
				return RIGHT_BEACON_BYTE;

			if ((recently_center_left_focus.current_state && recently_center_right_focus.current_state))
				return (LEFT_BEACON_BYTE|RIGHT_BEACON_BYTE);

		} while(recently_follow_left_force_field.current_state);

		already_90 = FALSE;
		orientation = get_gyro_angle();

		do
		{
			angle = orientation - get_gyro_angle();
			if (abs(angle) > 350)
			{
				set_docking_force_field_abort();
				return 0;
			}
			if (abs(angle) > 180)
			{
				if (angle > 0)
				{
					angle = 360 - angle;
				}
				else
				{
					angle = 360 + angle;
				}
			}

			dprintf(DEBUG_DOCK_FORCE_FIELD, "left in\r\n");

			if ((abs(angle) >= 90) && (recently_force_field_middle.current_state))
			{
				set_motor_vels(0, 0, ACCELERATION_MIN);
				break;
			}
			else
			{
				left_vel = 20;
				right_vel = 90;

				set_motor_vels(left_vel, right_vel, ACCELERATION_MIN);
			}

			if (recently_bump.current_state)
			{
				docking_force_field_orientation = (get_gyro_angle()+180);
				return 0;
			}

			if (recently_left_left.current_state)
				return LEFT_BEACON_BYTE;

			if (recently_right_left.current_state)
				return RIGHT_BEACON_BYTE;

			if ((recently_center_left_focus.current_state && recently_center_right_focus.current_state))
				return (LEFT_BEACON_BYTE|RIGHT_BEACON_BYTE);

		} while(!recently_follow_left_force_field.current_state);

		goto FOLLOW_FORCE_FIELD_LEFT;
	}

	return 0;
}

DOCK_FN_DECL(docking_force_field)
{
	S8 result = 0;
	U16 follow_reslut = 0;
	S16 angle = 0;
	S8 follow_force_field_count = 0;
	U32 turn_stop_time = 0;
	static volatile AM_LeftRight docking_force_field_dir = AM_LEFT;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_force_field\r\n");

	turn_stop_time = timer_ms();
	do
	{
		set_motor_vels(0, 0, ACCELERATION_MAX);
	}
	while (timer_elapsed(turn_stop_time) < 100);

	if (recently_follow_right_force_field.current_state)
	{
		docking_force_field_dir = AM_RIGHT;
	}
	else if (recently_follow_left_force_field.current_state)
	{
		docking_force_field_dir = AM_LEFT;
	}
	else
	{

		if (docking_force_field_dir == AM_LEFT)
		{
			docking_force_field_dir = AM_RIGHT;
			AM_GO_TO_PLACE(90,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
					TRUE, \
					CARE_CLIFF,result);
		}
		else
		{
			docking_force_field_dir = AM_LEFT;
			AM_GO_TO_PLACE(-90,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
					TRUE, \
					CARE_CLIFF,result);
		}
	}

DOCKING_FORCE_FIELD_FOLLOW:
	follow_reslut = follow_force_field(docking_force_field_dir);

	if (follow_reslut == 0)
	{
		if (follow_force_field_count > 2)
		{
			if (recently_right_near_dock.current_state)
			{
				AM_GO_TO_PLACE(90,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);

				DRIVE_GO(200,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);
			}
			else if (recently_left_near_dock.current_state)
			{
				AM_GO_TO_PLACE(-90,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);

				DRIVE_GO(200,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);
			}

			set_docking_force_field_abort();

			return;
		}
		else
		{
			follow_force_field_count++;
		}

		while(recently_bump.current_state){DRIVE_GO(-10,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);};

		if (docking_force_field_dir == AM_RIGHT)
		{
			docking_force_field_dir = AM_LEFT;

			angle = docking_force_field_orientation - get_gyro_angle();
			if (abs(angle) > 180)
			{
				if (angle > 0)
				{
					angle = 360 - angle;
				}
				else
				{
					angle = 360 + angle;
				}
			}
			AM_GO_TO_PLACE(180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
			TRUE, \
			CARE_CLIFF,result);
			dprintf(DEBUG_DOCK_FORCE_FIELD, "right to left\r\n");
		}
		else if (docking_force_field_dir == AM_LEFT)
		{
			docking_force_field_dir = AM_RIGHT;
			angle = docking_force_field_orientation - get_gyro_angle();
			if (abs(angle) > 180)
			{
				if (angle > 0)
				{
					angle = angle - 360;
				}
				else
				{
					angle = angle;
				}
			}
			AM_GO_TO_PLACE(-180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
			TRUE, \
			CARE_CLIFF,result);
			dprintf(DEBUG_DOCK_FORCE_FIELD, "left to right\r\n");
		}

		goto DOCKING_FORCE_FIELD_FOLLOW;
	}
	else if (follow_reslut != 0)
	{
		if (follow_reslut == (LEFT_BEACON_BYTE|RIGHT_BEACON_BYTE))
		{
			AM_GO_TO_PLACE(180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
			TRUE, \
			CARE_CLIFF,result);

			DRIVE_GO(600,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

			docking_find_set_orientation_direction();

			if (docking_force_field_dir == AM_RIGHT)
			{
				AM_GO_TO_PLACE(-75,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);
			}
			else
			{
				AM_GO_TO_PLACE(75,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);
			}

		}
		else if (follow_reslut == (RIGHT_BEACON_BYTE))
		{
			if (docking_force_field_dir == AM_LEFT)
			{
				AM_GO_TO_PLACE(-75,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);
			}
			else
			{
				AM_GO_TO_PLACE(75,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);
			}

			DRIVE_GO(800,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

			docking_find_set_orientation_direction();

			AM_GO_TO_PLACE(-90,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
					TRUE, \
					CARE_CLIFF,result);
		}
		else if (follow_reslut == (LEFT_BEACON_BYTE))
		{
			if (docking_force_field_dir == AM_LEFT)
			{
				AM_GO_TO_PLACE(-75,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);
			}
			else
			{
				AM_GO_TO_PLACE(75,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE, \
				CARE_CLIFF,result);
			}
			DRIVE_GO(800,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

			docking_find_set_orientation_direction();


			AM_GO_TO_PLACE(90,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
					TRUE, \
					CARE_CLIFF,result);

		}
		else
		{
			dprintf(DEBUG_DOCK_BEHAVIOR, "docking_force_field not direction\r\n");

			goto DOCKING_FORCE_FIELD_FOLLOW;
		}
	}

	docking_find_set_head_direction();

	docking_find_buoy_start_set();

	set_docking_force_field_abort();

	return ;
}

BOOLEAN docking_force_field_start_when(void)
{
	if (recently_force_field.current_state && \
		(!(recently_no_force_field.current_state)) && \
		(current_dock_behavior() != DOCKING_FIND_BUOY) && \
		(current_dock_behavior() != DOCKING_LEFT) && \
		(current_dock_behavior() != DOCKING_RIGHT) && \
		(current_dock_behavior() != DOCKING_GO_FORWARD) && \
		(current_dock_behavior() != DOCKING_CORRECT))
	{
		return TRUE;
	}
	else
		return FALSE;
}

void docking_force_field_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_FORCE_FIELD;
	dock_funtion.start_when = &docking_force_field_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_force_field_abort_when;
	dock_funtion.abort_code = &docking_force_field_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_force_field;

	register_dock_function(&dock_funtion);

	return;
}
/******************************* DOCK FORCE FIELD END *****************************/


/********************************** DOCK FIND BUOY ********************************/
/**
 * dock find buoy - 搜索F4或F8信号行为
 * NOTE:绕座行为退出后，重新搜索F4或F8信号
 * 触发条件: 绕座行为退出时，触发此行为
 * 退出条件: 无
 */
static BOOLEAN docking_find_buoy_start = FALSE;
static S64 docking_find_buoy_heading = 0;
static S64 docking_find_buoy_orientation = 0;
void set_docking_find_buoy_abort(void)
{
	docking_find_buoy_start = FALSE;
	return;
}

void docking_find_buoy_abort_code(void)
{
	docking_find_buoy_start = FALSE;
	return;
}

void docking_find_buoy_start_set(void)
{
	docking_find_buoy_start = TRUE;
	return;
}

void docking_find_set_orientation_direction(void)
{
	docking_find_buoy_orientation = get_gyro_angle() + 180;
	docking_find_buoy_orientation = docking_find_buoy_orientation;

	return;
}

void docking_find_set_head_direction(void)
{
	docking_find_buoy_heading = get_gyro_angle();
	docking_find_buoy_heading = docking_find_buoy_heading;

	return;
}

DOCK_FN_DECL(docking_find_buoy)
{
	S64 orientation = 0;
	S16 angle = 0;
	S8 result = 0;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_find_buoy\r\n");

	orientation = get_gyro_angle();
	angle = docking_find_buoy_heading - orientation;

	if (abs(angle) > 180)
	{
		if (angle > 0)
		{
			angle = angle - 360;
		}
		else
		{
			angle = 360 + angle;
		}
	}

	AM_GO_TO_PLACE(angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,\
				(CARE_CLIFF|CARE_BUMP),result);
	if (result != 0)
	{
		if (angle > 0)
		{
			angle = angle - 360;
		}
		else
		{
			angle = 360 + angle;
		}
		AM_GO_TO_PLACE(angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,\
				CARE_CLIFF,result);
		if (result != 0)
		{
			set_docking_find_buoy_abort();
		}
	}

	angle = (docking_find_buoy_orientation - docking_find_buoy_heading);
	if (abs(angle) > 180)
	{
		if (angle > 0)
		{
			angle = angle - 360;
		}
		else
		{
			angle = 360 + angle;
		}
	}

	if (angle > 0)
	{
		angle = 135;
	}
	else
	{
		angle = -135;
	}

	AM_GO_TO_PLACE(angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	AM_GO_TO_PLACE(-angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,\
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	DRIVE_GO(300,FORWARDSPEED, \
		(!recently_left_left.current_state && !recently_right_right.current_state && \
		!recently_left_right.current_state && !recently_right_left.current_state),\
		(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	AM_GO_TO_PLACE(-angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,\
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	DRIVE_GO(300,FORWARDSPEED, \
		(!recently_left_left.current_state && !recently_right_right.current_state && \
		!recently_left_right.current_state && !recently_right_left.current_state),\
		(CARE_CLIFF|CARE_BUMP),result);

	if (angle > 0)
	{
		AM_GO_TO_PLACE(180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,\
				CARE_CLIFF,result);
	}
	else
	{
		AM_GO_TO_PLACE(-180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,\
				CARE_CLIFF,result);
	}
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	DRIVE_GO(900,FORWARDSPEED, \
		(!recently_left_left.current_state && !recently_right_right.current_state && \
		!recently_left_right.current_state && !recently_right_left.current_state),\
		(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(-angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	AM_GO_TO_PLACE(angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,\
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	DRIVE_GO(300,FORWARDSPEED, \
		(!recently_left_left.current_state && !recently_right_right.current_state && \
		!recently_left_right.current_state && !recently_right_left.current_state),\
		(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(-angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED, \
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	AM_GO_TO_PLACE(angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,\
				TRUE,\
				CARE_CLIFF,result);
	if (result != 0)
	{
		set_docking_find_buoy_abort();
	}

	set_docking_find_buoy_abort();

	return ;
}

BOOLEAN docking_find_buoy_run_when(void)
{
	if (docking_find_buoy_start == TRUE)
	{
		return TRUE;
	}
	else
		return FALSE;
}

void docking_find_buoy_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_FIND_BUOY;
	dock_funtion.start_when = NULL;
	dock_funtion.run_when = &docking_find_buoy_run_when;
	dock_funtion.abort_when = NULL;
	dock_funtion.abort_code = &docking_find_buoy_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_find_buoy;

	register_dock_function(&dock_funtion);

	return;
}
/******************************** DOCK FIND BUOY END ******************************/


/********************************* DOCK FORCE FIELD *******************************/
/**
 * dock only field - 只有圆泡收到信号时，搜索有用信号
 * NOTE:此行为原地转一圈，以搜索是否存在有用的信号
 * 触发条件: 只有圆泡接收到信号时触发
 * 退出条件: 无
 */
static BOOLEAN docking_only_force_field_abort = FALSE;
static S16 docking_only_force_field_angle = 360;
void set_docking_only_force_field_abort(void)
{
	docking_only_force_field_abort = TRUE;
	return;
}

BOOLEAN docking_only_force_field_abort_when(void)
{
	if(docking_only_force_field_abort != FALSE)
		return TRUE;
	else
		return FALSE;
}

void docking_only_force_field_abort_code(void)
{
	docking_only_force_field_abort = FALSE;
	return;
}

DOCK_FN_DECL(docking_only_force_field)
{
	S8 result = 0;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_only_force_field\r\n");

	AM_GO_TO_PLACE(docking_only_force_field_angle,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,\
				0,result);

	DRIVE_GO(100,FORWARDSPEED,TRUE,0,result);

	set_docking_only_force_field_abort();

	return ;
}

BOOLEAN docking_only_force_field_start_when(void)
{
	if ((check_back_left_beacon() == 1) || \
		(check_back_right_beacon() == 1) || \
		((last_dock_behavior() == DOCKING_CORRECT) && \
		(current_dock_behavior() == DOCKING_LINE)))
	{
		docking_only_force_field_angle = 360;

		return TRUE;
	}
	else
		return FALSE;
}

void docking_only_force_field_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_ONLY_FORCE_FIELD;
	dock_funtion.start_when = &docking_only_force_field_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_only_force_field_abort_when;
	dock_funtion.abort_code = &docking_only_force_field_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_only_force_field;

	register_dock_function(&dock_funtion);

	return;
}
/******************************* DOCK FORCE FIELD END *****************************/


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

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_right\r\n");

	/*if (recently_docking_go_forward.current_state)
	{
		already_mid = TRUE;
	}*/

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

	set_motor_vels(left_vel, right_vel, ACCELERATION_NON_EMERGENCY);
	return ;
}

BOOLEAN docking_right_run_when(void)
{
	if (recently_docking_right.current_state)
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

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_left\r\n");

	/*if (recently_docking_go_forward.current_state)
	{
		already_mid = TRUE;
	}*/

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

	set_motor_vels(left_vel, right_vel, ACCELERATION_NON_EMERGENCY);

	return ;
}

BOOLEAN docking_left_run_when(void)
{
	if (recently_docking_left.current_state)
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
	if (recently_docking_go_forward.current_state)
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

	if (recently_docking_left.current_state || \
		recently_docking_right.current_state || \
		docking_correct_abort)
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

	/*if (recently_docking_go_forward.current_state)
	{
		already_mid = TRUE;
	}*/

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


/************************************ DOCK LINE ***********************************/
/**
 * dock line - 直行行为
 * NOTE:此行为与dock line bounce行为配合，实现随机功能
 * 触发条件: 优先级最低，触发条件永远为真
 * 退出条件: 无
 */
DOCK_FN_DECL(docking_line)
{
	S8 result = 0;
	U16 angle = 0;
	BOOLEAN slow_speed = FALSE;

	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_line\r\n");

	do
	{
		if (((last_dock_behavior() == DOCKING_LEFT_RIGHT) || \
		(last_dock_behavior() == DOCKING_SUCCESS) || \
		(last_dock_behavior() == DOCKING_DOCK_AHEAD) || \
		(last_dock_behavior() == DOCKING_BOUNCE)) &&
		(slow_speed == FALSE))
		{
			slow_speed = TRUE;
			DRIVE_GO(20,FORWARD_NEAR_SLOW_SPEED,TRUE,0,result);
		}
		else
		{
			DRIVE_GO(5000,FORWARDSPEED, \
		       TRUE,\
		       (CARE_CLIFF|CARE_BUMP),result);
			angle = get_random();

			AM_GO_TO_PLACE(angle,DOCKING_TRUN_SLOWEST_SPEED,\
		    	DOCKING_TRUN_SLOWEST_SPEED,TRUE,CARE_CLIFF,result);
		}
	}
	while (1);
}

BOOLEAN docking_line_run_when(void)
{
	return TRUE;
}

void docking_line_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_LINE;
	dock_funtion.start_when = NULL;
	dock_funtion.run_when = &docking_line_run_when;
	dock_funtion.abort_when = NULL;
	dock_funtion.abort_code = NULL;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_line;

	register_dock_function(&dock_funtion);

	return;
}
/************************************ DOCK LINE ***********************************/


void dock_get_random_count(void)
{
#if 0
	static U16 count = 0;

	if (count > 500)
	{
		printf("random_count %d\r\n", docking_state.random_behavior_count);
		count = 0;
	}
	else
		count++;
#endif
#ifdef IR_WIRELESS
	ir_send();
#endif
	if ((current_dock_behavior() == DOCKING_LINE_BOUNCE) || (current_dock_behavior() == DOCKING_LINE))
		docking_state.random_current_count++;
	else if (docking_state.random_current_count > 0)
		docking_state.random_current_count--;

	if (docking_state.random_current_count <= 0)
		docking_state.random_behavior_count = 0;
	else
		docking_state.random_behavior_count = docking_state.random_current_count/10;

	return;
}

dock_config_t* dock_new_init(void)
{
	dock_success_register();
	docking_bounce_register();
	docking_ahead_register();
	docking_left_right_register();
	docking_force_field_register();
	docking_find_buoy_register();
	docking_only_force_field_register();
	dock_right_register();
	dock_left_register();
	docking_go_forward_register();
	dock_correct_register();
	docking_avoid_obstacle_register();
	docking_line_bounce_register();
	docking_line_register();

	register_debouncer(&recently_signal);
	register_debouncer(&recently_near_dock);
	register_debouncer(&recently_near_dock_1);
	register_debouncer(&recently_docking_left);
	register_debouncer(&recently_docking_right);
	register_debouncer(&recently_left_left);
	register_debouncer(&recently_right_right);
	register_debouncer(&recently_left_backleft);
	register_debouncer(&recently_right_backright);
	register_debouncer(&recently_right_backleft);
	register_debouncer(&recently_left_backright);
	register_debouncer(&recently_docking_go_forward_right);
	register_debouncer(&recently_docking_go_forward_left);
	register_debouncer(&recently_docking_go_forward_onlyright);
	register_debouncer(&recently_docking_go_forward_onlyleft);
	register_debouncer(&recently_docking_go_forward);
	register_debouncer(&recently_force_field);
	register_debouncer(&recently_no_force_field);
	register_debouncer(&recently_center_left_focus);
	register_debouncer(&recently_center_right_focus);
	register_debouncer(&recently_follow_left_force_field);
	register_debouncer(&recently_follow_right_force_field);
	register_debouncer(&recently_left_near_dock);
	register_debouncer(&recently_right_near_dock);
	register_debouncer(&recently_left_right);
	register_debouncer(&recently_right_left);
	register_debouncer(&recently_force_field_middle);
	register_debouncer(&recently_bump);

	register_dock_signals(&robot_get_dock_signals);
	register_random_conut(&dock_get_random_count);

	dock_config.max_ir_chan = IR_MAX_RECV;

	/* 圆泡看到圆泡信号 */
	dock_config.dock_avoid_chan = 0;
	dock_config.dock_avoid_chan = ((0x1<<IR_LOCAL_MID_LEFT)|(0x1<<IR_LOCAL_MID_RIGHT));
	/* 双目看到圆泡信号 */
	dock_config.binocular_see_avoid_chan = 0;
	dock_config.binocular_see_avoid_chan = ((0x1<<IR_LOCAL_MID_LEFT)|(0x1<<IR_LOCAL_MID_RIGHT));

	dock_config.aovw_chan = 0;
	dock_config.aovw_chan = ((0x1<<IR_LOCAL_MID_LEFT)|(0x1<<IR_LOCAL_MID_RIGHT)|\
									(0x1<<IR_LOCAL_LEFT)|(0x1<<IR_LOCAL_RIGHT));
	dock_config.dock_signals_type.dock_closed = DOCK_CLOSE_BEACON;
	dock_config.dock_signals_type.left_signal = LEFT_BEACON_BYTE;
	dock_config.dock_signals_type.right_signal = RIGHT_BEACON_BYTE;
	dock_config.dock_signals_type.center_signal = 0xff;
	dock_config.dock_signals_type.RESERVE1 = 0xff;
	dock_config.dock_signals_type.RESERVE2 = 0xff;
	dock_config.dock_signals_type.aovw_signal = AOVW_BYTE;

	dock_config.success_behavior_id = DOCKING_SUCCESS;
	dock_config.first_behavior_id = DOCKING_LINE;

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


	docking_parameter_init();

	dock_core_enable();
	set_dock_leave_dockagain(0);
	return;
}

BOOLEAN dock_new_end(U8 *uTerm)
{
	if ((docking_state.dock_finished == FALSE) && \
		(docking_state.random_behavior_count < DOCKINT_RANDOM_THRESHOLD))
		return FALSE;

	if (docking_state.dock_finished == TRUE)
		*uTerm = DOCKING_SUCESS;
	else if (docking_state.random_behavior_count >= DOCKINT_RANDOM_THRESHOLD)
		*uTerm = DOCKING_FAIL;

	dock_core_disable();
	clear_debouncer();

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
#endif
