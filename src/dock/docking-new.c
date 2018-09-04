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
#define DOCKING_NEAR_SLOWEST_SPEED    (60)
#define DOCKING_NEAR_SLOW_SPEED       (100)
#define DOCKING_SLOWEST_SPEED         (130)
#define DOCKING_SLOW_SPEED            (200)
#define FORWARD_NEAR_SLOW_SPEED       (90)
#define FORWARD_SLOW_SPEED            (180)
#define FORWARDSPEED                  (240)

#define VERIFY_HOLD_CNT               (2)

#define CARE_BUMP                     (1)
#define CARE_CLIFF                    (2)

static DockingState docking_state =
{
	0,
	FALSE,
	0,
	0,
	0x0FFF,
};

static dock_config_t dock_config;
U8 go_forward_flag=FALSE;

BOOLEAN docking_left_right_start_when(void);
BOOLEAN docking_left_run_when(void);
BOOLEAN docking_right_run_when(void);
void docking_find_set_orientation_direction(void);
void docking_find_set_head_direction(void);

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
	U32 time;
	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_success\r\n");
  robot_sidebrush_vols_set(0);      
	go_forward_flag=FALSE;
  do
  {
          set_motor_vels(0, 0, ACCELERATION_MAX);
          get_motor_speeds(&vl_meas, &vr_meas);
  }
  while ((vl_meas > 0) || (vr_meas > 0));
	
	time=timer_ms();
	while(timer_elapsed(time)<1000);
		docking_state.state_cnt++;

	if (!charging_detect())
	{
		dprintf(DEBUG_DOCK_BEHAVIOR, "docking_verify_charger fail \r\n");
		docking_state.state_cnt = 0;
		DRIVE_GO(-200,FORWARDSPEED,(!charging_detect()),(CARE_CLIFF|CARE_BUMP),result);
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
	go_forward_flag=FALSE;
	
	dprintf(DEBUG_DOCK_BEHAVIOR, "docking_bounce\r\n");

	while((get_bump_state() !=0) || (get_cliff_state() != 0)){};

	DRIVE_GO(-50,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

	DRIVE_GO(600,FORWARDSPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);

	AM_GO_TO_PLACE(180,DOCKING_TRUN_SLOWEST_SPEED,DOCKING_TRUN_SLOWEST_SPEED,\
		TRUE, \
		CARE_CLIFF,result);

	set_docking_bounce_abort();

	return ;
}

BOOLEAN docking_bounce_start_when(void)
{
	if (((get_bump_state() !=0) || (get_cliff_state() != 0)) && \
		(((recently_near_dock.current_state) && \
		((recently_center_left_focus.current_state && recently_center_right_focus.current_state)) && \
		(current_dock_behavior() != DOCKING_FORCE_FIELD)) || \
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
	if(recently_docking_left_midright.current_state)
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
	if(recently_docking_right_midleft.current_state)
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
	go_forward_flag=TRUE;

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
	if(recently_docking_left_midright.current_state&&recently_docking_right_midright.current_state&&\
		recently_docking_left_midleft.current_state&&recently_docking_right_midleft.current_state)
		return TRUE;
	else if((!recently_docking_left_midright.current_state)&&recently_docking_right_midright.current_state&&\
					recently_docking_left_midleft.current_state&&(!recently_docking_right_midleft.current_state))
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
/********************************** FIND_MIDDLE ********************************/
static BOOLEAN findmiddle_abort_flag=FALSE;
static bool findmiddle_flag=false;
U8 find_mid_dir=0;
void findmiddle_abort(void)
{
      findmiddle_abort_flag=TRUE;

	return;
}
DOCK_FN_DECL(fine_middle)
{
      S8 result = 0;
      findmiddle_flag=true;
			set_motor_vels(0, 0,ACCELERATION_MIN);
      //right
     if(find_mid_dir==1)
      {
        AM_GO_TO_PLACE(-180,150,150,\
          (!(docking_left_run_when()||docking_right_run_when())),\
            CARE_CLIFF,result);
        AM_GO_TO_PLACE(-20,150,150,\
          (!(docking_go_forward_run_when()||docking_left_run_when())),\
            CARE_CLIFF,result);
      }
      else if(find_mid_dir==2)
      {
        AM_GO_TO_PLACE(180,150,150,\
          (!(docking_left_run_when()||docking_right_run_when())),\
            CARE_CLIFF,result);
        AM_GO_TO_PLACE(20,150,150,\
          (!(docking_go_forward_run_when()||docking_right_run_when())),\
            CARE_CLIFF,result);
        
      }
      findmiddle_abort();
	return;
}

BOOLEAN find_middle_start_when(void)
{
		if(((current_dock_behavior()==DOCKING_LEFT)||(current_dock_behavior()==DOCKING_RIGHT)||\
			(current_dock_behavior()==DOCKING_CORRECT))&&\
			((recently_right_centerright.current_state && recently_left_centerright.current_state)||\
			(recently_left_centerleft.current_state && recently_right_centerleft.current_state)))
			{
					return FALSE;
			}
            
	  if(((recently_right_right.current_state && recently_left_right.current_state)||\
	     ( recently_left_backright.current_state&&recently_right_backright.current_state)||\
	      (recently_right_centerright.current_state && recently_left_centerright.current_state)))
	  {
			    find_mid_dir=1;
			    return  TRUE;
	  }
	  else if (((recently_left_left.current_state && recently_right_left.current_state)||\
	    ( recently_right_backleft.current_state&&recently_left_backleft.current_state)||\
	      (recently_left_centerleft.current_state && recently_right_centerleft.current_state)))  
	  {
			  find_mid_dir=2;
			    return  TRUE;
	  }
	   else 
	  return FALSE;
}


BOOLEAN find_middle_abort_when(void)
{
	if (findmiddle_abort_flag!=FALSE)
		return TRUE;
	else
		return FALSE;
}

void find_middle_abort_code(void)
{

	findmiddle_abort_flag = FALSE;
	find_mid_dir=0;

	return;
}
void fine_middle_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = FIND_MIDDLE;
	dock_funtion.start_when = &find_middle_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &find_middle_abort_when;
	dock_funtion.abort_code = &find_middle_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = fine_middle;

	register_dock_function(&dock_funtion);

	return;
}

/******************************** FIND_MIDDLE END ******************************/
/************************************ DOCKING_RIGHT_LEFT_FIELD ***********************************/
/**

 */
static BOOLEAN docking_right_left_field_abort=FALSE;
static S8 robot_position=0;
void set_docking_right_left_field_abort(void)
{
  docking_right_left_field_abort=TRUE;
	return;
}
DOCK_FN_DECL(docking_right_left_field)
{
  S8 result;
  AM_GO_TO_PLACE(80*robot_position,100,100,\
 (!(check_recently_right_centerright()||check_recently_left_centerleft())),\
  CARE_CLIFF,result);
	DRIVE_GO(400,DOCKING_SLOW_SPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);
  set_docking_right_left_field_abort();
  return;
}


void docking_right_left_field_abort_code(void)
{
  docking_right_left_field_abort=FALSE;
  robot_position=0;
	return;
}

BOOLEAN docking_right_left_field_abort_when(void)
{
  if(TRUE==docking_right_left_field_abort)
    return TRUE;
  else 
    return FALSE;
}
BOOLEAN docking_right_left_field_start_when(void)
{
//  if(recently_left_midright.current_state||recently_right_midleft.current_state||recently_right_midright.current_state||\
//    recently_left_midleft.current_state)
//  {
//      return FALSE;
//  }

    if(check_recently_left_left()||check_recently_left_backleft())
    {
        robot_position=1;
				return TRUE;
    }
    else if(check_recently_right_right()||check_recently_right_backright())
    {
        robot_position=-1;
				return TRUE;
    }
	  else
	    return FALSE;

}
void docking_right_left_field_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = DOCKING_RIGHT_LEFT_FIELD;
	dock_funtion.start_when = &docking_right_left_field_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &docking_right_left_field_abort_when;
	dock_funtion.abort_code = &docking_right_left_field_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = docking_right_left_field;

	register_dock_function(&dock_funtion);

	return;
}
/************************************ DOCKING_RIGHT_LEFT_FIELD ***********************************/

/************************************ ROLL DOCKING ***********************************/
static bool roll_docking_abort_flag =false;
static U8 roll_diretion = AM_RIGHT;
static float roll_angle_division=2;
static  U8 bump_count=0;
static  U32 roll_docking_abort_time=0;
BumpState bump_state=BUMP_MASK_NONE;
CliffState cliff_state=BUMP_MASK_NONE;			

void roll_docking_abort(void)
{
		  roll_docking_abort_flag=true;
			return;
}
BOOLEAN roll_docking_start_when(void)
{            
				if((current_dock_behavior()==FIND_MIDDLE)||(last_dock_behavior()==ROLL_DOCKING)||\
					(timer_elapsed(roll_docking_abort_time)<5000)||(current_dock_behavior()==ROLL_DOCKING)||\
					go_forward_flag)
					return FALSE;
        else if(recently_near_dock_1.current_state)
         {            
               return TRUE;
         }
         else 
          return FALSE;
}
DOCK_FN_DECL(roll_docking)
{
    S8 result;
    int16_t left_motor;
    int16_t right_motor;
    bool roll_in_signal = FALSE;
    U8 roll_no_signal_count=0;
    U8 last_roll_diretion;
    int angle;
		uint32_t bump_time;
    if(last_dock_behavior()==DOCKING_LINE)  //以尽量短的时间煞住
    {
				do
				{
						set_motor_vels(-10, -10,60000);
						get_motor_speeds(&left_motor, &right_motor);
				}
			  while ((left_motor > 0) || (right_motor> 0));
    }
    roll_angle_division=2.5;
    last_roll_diretion=roll_diretion;
    //在信号区触发尝试用信号判断机器左右
    if(recently_force_field_midleft.current_state||recently_force_field_left.current_state||recently_force_field_centerleft.current_state)
    {
        if(recently_force_field_midleft.current_state)
        {
            if(recently_docking_left_midleft.current_state||(recently_docking_left_midright.current_state&&\
              (!recently_left_left.current_state)))
            {
               roll_diretion = AM_LEFT;
               roll_angle_division=-2.5;
            }
//            else if(recently_left_centerleft.current_state&&recently_docking_left_midleft.current_state)
//            {
//               roll_diretion = AM_LEFT;
//               roll_angle_division=-3.6;
//            }
//            else if(!recently_docking_left_midleft.current_state&&recently_left_centerleft.current_state)
//            {
//               roll_diretion = AM_LEFT;
//               roll_angle_division=-6;
//            }
        }
        else if((recently_force_field_centerleft.current_state||recently_force_field_left.current_state)&&\
					(!recently_force_field_midright.current_state))
        {
           roll_diretion = AM_LEFT;
           roll_angle_division=-4.5;
        }
    }
    else if(recently_force_field_midright.current_state||recently_force_field_right.current_state||recently_force_field_centerright.current_state)
    {
        if(recently_force_field_midright.current_state)
        {
            if(recently_docking_right_midleft.current_state||(recently_docking_right_midright.current_state&&\
              (!recently_right_right.current_state)))
            {
               roll_diretion = AM_RIGHT;
               roll_angle_division=2.5;
            }
//            else if(recently_right_centerright.current_state&&recently_docking_right_midright.current_state)
//            {
//               roll_diretion = AM_RIGHT;
//               roll_angle_division=3.6;
//            }
//            else if(!recently_docking_right_midright.current_state&&recently_right_centerright.current_state)
//            {
//               roll_diretion = AM_RIGHT;
//               roll_angle_division=6;
//            }
        }
        else if((recently_force_field_right.current_state||recently_force_field_centerright.current_state)&&\
					(!recently_force_field_midright.current_state))
        {
            roll_diretion = AM_RIGHT;
            roll_angle_division=4.5;
        }
    }
     if(bump_state||cliff_state)
    {
        bump_count++;

        if(bump_count>=4)
        {
            bump_count=0;
              AM_GO_TO_PLACE(45,DOCKING_TRUN_SLOWEST_SPEED,\
              DOCKING_TRUN_SLOWEST_SPEED,\
              TRUE,\
              CARE_CLIFF,result);
            roll_docking_abort();
        }
        if(last_roll_diretion == AM_LEFT)
        {
          roll_angle_division=1;
          roll_diretion = AM_RIGHT;
          
        }
        else if(last_roll_diretion == AM_RIGHT)
        {  
          roll_angle_division=1;
          roll_diretion = AM_LEFT;  
          
        }

       if(bump_state==BUMP_FRONT_LEFT) 
       {
           roll_angle_division = -1.2;
           //   roll_diretion = AM_RIGHT;
       }
       else if(bump_state==BUMP_FRONT_RIGHT) 
       {
           roll_angle_division = 1.2;
            //  roll_diretion = AM_LEFT;  
       }
       bump_state = BUMP_MASK_NONE;
			 cliff_state = CLIFF_NONE;
      // printf("bump_state() %d PY_bump %d LT_bump %d\r\n",get_bump_state(),PY_bump,LT_bump);
       while(get_bump_state()||get_cliff_state()) {};
    }
    //掉头过程中遇到bump 
    do
    {
      angle=get_gyro_angle();
      AM_GO_TO_PLACE((S16)(180/roll_angle_division),DOCKING_TRUN_SLOWEST_SPEED,\
              DOCKING_TRUN_SLOWEST_SPEED,\
              TRUE,\
              CARE_CLIFF,result);
      if(result)
      {
        AM_GO_TO_PLACE((S16)((180/roll_angle_division)-(get_gyro_angle()-angle)),DOCKING_TRUN_SLOWEST_SPEED,\
        DOCKING_TRUN_SLOWEST_SPEED,\
        TRUE,\
        CARE_CLIFF,result);
      
      }
    }while(result);

    //绕座
    angle = get_gyro_angle();
    do 
    {
      //绕座过程中有收到某些信号
//      if(recently_right_midright.current_state||recently_right_right.current_state||\
//        recently_right_backright.current_state||recently_left_midleft.current_state||\
//        recently_left_left.current_state||recently_left_backleft.current_state )
      if(recently_right_right.current_state||\
        recently_right_backright.current_state||\
        recently_left_left.current_state||recently_left_backleft.current_state )

      {
          roll_in_signal=TRUE;
          roll_no_signal_count=0;
          angle=get_gyro_angle();
      }

      if(roll_diretion == AM_LEFT)
      {
        //绕座过程中没有收到信号绕的角度又超过125，调整角度继续绕。
//       if((!roll_in_signal)&&(abs(get_gyro_angle()-angle)>125))
//       {
//         roll_no_signal_count++;
//         AM_GO_TO_PLACE(-110,DOCKING_TRUN_SLOWEST_SPEED,\
//              DOCKING_TRUN_SLOWEST_SPEED,\
//              TRUE,\
//              CARE_CLIFF,result);
//         angle=get_gyro_angle();
//       }
//        if(roll_no_signal_count>=2)
//       {
//            AM_GO_TO_PLACE(-90,DOCKING_TRUN_SLOWEST_SPEED,\
//            DOCKING_TRUN_SLOWEST_SPEED,\
//            TRUE,\
//            CARE_CLIFF,result);
//            roll_docking_abort();
//       }
       if(recently_near_dock_1.current_state)
       {

          left_motor=120;
          right_motor=50;
       }
       else
       {
          left_motor=60;
          right_motor=120;
       
       }
      }
      
      else if(roll_diretion == AM_RIGHT)
      {
       
//       if((!roll_in_signal)&&(abs(get_gyro_angle()-angle)>125))
//       {
//         roll_no_signal_count++;
//         AM_GO_TO_PLACE(110,DOCKING_TRUN_SLOWEST_SPEED,\
//              DOCKING_TRUN_SLOWEST_SPEED,\
//              TRUE,\
//              CARE_CLIFF,result);
//         angle=get_gyro_angle();
//       }
//        if(roll_no_signal_count>=2)
//       {
//            AM_GO_TO_PLACE(90,DOCKING_TRUN_SLOWEST_SPEED,\
//            DOCKING_TRUN_SLOWEST_SPEED,\
//            TRUE,\
//            CARE_CLIFF,result);
//            roll_docking_abort();
//       }
       if(recently_near_dock_1.current_state)
       {
          left_motor=50;
          right_motor=120;
       }
       else
       {
          left_motor=120;
          right_motor=60;
       
       }
      }
    set_motor_vels(left_motor, right_motor, ACCELERATION_MAX);
    }while((!get_bump_state())&&(!get_cliff_state())&&(!(recently_left_left.current_state&&recently_right_left.current_state))&&\
      (!(recently_right_right.current_state&&recently_left_right.current_state))&&\
      (!(recently_right_centerright.current_state&&recently_left_centerright.current_state))&&\
      (!(recently_right_centerleft.current_state&&recently_left_centerleft.current_state)));
    bump_state=get_bump_state();
		cliff_state=get_cliff_state();
    if(bump_state||cliff_state)
		bump_time = timer_ms();
		while(get_bump_state()||get_cliff_state()||(timer_elapsed(bump_time)<500)){};
		
    roll_in_signal=FALSE;
    roll_no_signal_count=0;
		if((recently_right_centerright.current_state&&recently_left_centerright.current_state)||\
			(recently_right_centerleft.current_state&&recently_left_centerleft.current_state))
			{
			if(roll_diretion == AM_RIGHT)
				{
						DRIVE_GO(100,100, \
		       (!(recently_right_left.current_state)),\
		       (CARE_CLIFF|CARE_BUMP),result);
									AM_GO_TO_PLACE(-120,80,80,\
            (!((docking_go_forward_run_when()||docking_left_run_when()))),\
            CARE_CLIFF,result);
				}
		  if(roll_diretion == AM_LEFT)
				{
						DRIVE_GO(100,100, \
		       (!(recently_left_right.current_state)),\
		       (CARE_CLIFF|CARE_BUMP),result);
						AM_GO_TO_PLACE(120,80,80,\
            (!(docking_go_forward_run_when()||docking_right_run_when())),\
            CARE_CLIFF,result);
				}
			roll_docking_abort();

		}
		if((recently_left_left.current_state&&recently_right_left.current_state&&(roll_diretion == AM_LEFT)))
			{
			AM_GO_TO_PLACE(120,80,80,\
            (!(docking_go_forward_run_when()||docking_right_run_when())),\
            CARE_CLIFF,result);
			roll_docking_abort();
		}
		else if((recently_right_right.current_state&&recently_left_right.current_state&&(roll_diretion == AM_RIGHT)))
			{
			
			AM_GO_TO_PLACE(-120,80,80,\
            (!((docking_go_forward_run_when()||docking_left_run_when()))),\
            CARE_CLIFF,result);
			roll_docking_abort();
		}

    return;
}



BOOLEAN roll_docking_abort_when(void)
{
    if(roll_docking_abort_flag == TRUE)
      return TRUE;
    else
        return FALSE;
}

void roll_docking_abort_code(void)
{
   roll_docking_abort_time=timer_ms();
   bump_count=0;
   roll_diretion = AM_RIGHT;
   roll_angle_division=2;
   roll_docking_abort_flag =FALSE;
	 bump_state=BUMP_MASK_NONE;
   cliff_state=BUMP_MASK_NONE;	
	return;
}
void roll_docking_register(void)
{
	Dock_Data dock_funtion;

	dock_funtion.priorty = ROLL_DOCKING;
	dock_funtion.start_when = &roll_docking_start_when;
	dock_funtion.run_when = NULL;
	dock_funtion.abort_when = &roll_docking_abort_when;
	dock_funtion.abort_code = &roll_docking_abort_code;
	dock_funtion.last_start_state = FALSE;
	dock_funtion.current_function = roll_docking;

	register_dock_function(&dock_funtion);

	return;
}
/************************************ ROLL DOCKING ***********************************/


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
			DRIVE_GO(20,FORWARD_NEAR_SLOW_SPEED,TRUE,(CARE_CLIFF|CARE_BUMP),result);
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
	dock_right_register();
	dock_left_register();
	docking_go_forward_register();
	dock_correct_register();
//	docking_avoid_obstacle_register();
	docking_line_bounce_register();
	docking_line_register();
	roll_docking_register();
	fine_middle_register();
	docking_right_left_field_register();


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
	register_debouncer(&recently_docking_left_midright);
	register_debouncer(&recently_docking_right_midright);
	register_debouncer(&recently_docking_left_midleft);
	register_debouncer(&recently_docking_right_midleft);
	register_debouncer(&recently_center_left_focus);
	register_debouncer(&recently_center_right_focus);
	register_debouncer(&recently_left_right);
	register_debouncer(&recently_right_left);
	register_debouncer(&recently_bump);
	register_debouncer(&recently_right_centerright);
	register_debouncer(&recently_right_centerleft);
	register_debouncer(&recently_left_centerright);
	register_debouncer(&recently_left_centerleft);
	register_debouncer(&recently_force_field_right);
	register_debouncer(&recently_force_field_left);
	register_debouncer(&recently_force_field_midright);
	register_debouncer(&recently_force_field_midleft);
	register_debouncer(&recently_force_field_centerright);
	register_debouncer(&recently_force_field_centerleft);


	register_dock_signals(&robot_get_dock_signals);
	register_random_conut(&dock_get_random_count);

	dock_config.max_ir_chan = IR_MAX_RECV;

	/* 圆泡看到圆泡信号 */
	dock_config.dock_avoid_chan = 0;
	dock_config.dock_avoid_chan = ((0x1<<IR_LOCAL_MID_LEFT)|(0x1<<IR_LOCAL_MID_RIGHT)|(0x1<<IR_LOCAL_CENTER_LEFT)|\
																	(0x1<<IR_LOCAL_CENTER_RIGHT));
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
