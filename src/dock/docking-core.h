#ifndef _DOCKING_CORE_H_
#define _DOCKING_CORE_H_

typedef BOOLEAN (*judge_condition)(void);
typedef void (*abort_code)(void);
typedef U8 (*get_dock_signals)(U8 index);
typedef void (*get_random_count)(void);
typedef void (*dock_function)(void);
typedef void (*set_context)(BOOLEAN value);
#define DOCK_FN_DECL(a)  void a(void)

typedef struct
{
  dock_function current_function;
  judge_condition start_when;
  judge_condition run_when;
  judge_condition abort_when;
  abort_code abort_code;
  BOOLEAN last_start_state;
  S8 priorty;
}Dock_Data ;

typedef struct
{
  judge_condition predicate;
  U16 trigger_on;
  U16 trigger_off;
  U16 on_count;
  U16 off_count;
  BOOLEAN current_state;
  set_context set_dock_context;
}Debouncer_Data;

typedef struct
{
  U8 dock_closed;
  U8 left_signal;
  U8 right_signal;
  U8 center_signal;
  U8 RESERVE1;
  U8 RESERVE2;
  U8 aovw_signal;
}Dock_Ir_Signal;

typedef struct
{
	U8 max_ir_chan;
	U16 dock_avoid_chan;
	U16 binocular_see_avoid_chan;
	U16 aovw_chan;
	Dock_Ir_Signal dock_signals_type;
	U8 success_behavior_id;
	U8 first_behavior_id;
	U8 dock_again;
	U8 RESERVE1;
	U8 RESERVE2;
	U8 RESERVE3;
}dock_config_t;

#define AM_GO_TO_PLACE(rotation_angle,l_speed,r_speed,condition,care_bump,result)       \
{                                                                                      \
	S16 diff_angle = 0;                                                                \
	U32 turn_stop_time = 0;                                                            \
	                                                                                   \
	robot_turn(TRUE,rotation_angle,l_speed,r_speed,care_bump,&result);                 \
	do                                                                                 \
	{																				   \
		diff_angle = robot_turn(FALSE,rotation_angle,l_speed,r_speed,care_bump,&result); \
	}                                                                                  \
	while((diff_angle < abs(rotation_angle)) && condition && (result == 0));          \
	turn_stop_time = timer_ms();                                                       \
	do                                                                                 \
	{                                                                                  \
		set_motor_vels(0, 0, ACCELERATION_MAX);                                        \
	}                                                                                  \
	while (timer_elapsed(turn_stop_time) < 200);                                       \
}

#define DRIVE_GO(distance,speed,condition,care_bump,result)                            \
{                                                                                      \
	S16 diff_distance = 0;	                                                           \
	                                                                                   \
	robot_drive_go(TRUE,distance,speed,care_bump, &result);                             \
	do                                                                                 \
	{                                                                                  \
		 diff_distance = robot_drive_go(FALSE,distance,speed,care_bump, &result);       \
	}                                                                                  \
	while (((diff_distance) < abs(distance)) && condition && (result == 0));         \
}

#endif //_DOCKING_CORE_H_
