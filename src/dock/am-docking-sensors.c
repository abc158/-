//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "am-docking-new.h"
#include "am-docking-sensors.h"
#include "ui-ir.h"
#include "sensor/sensor.h"
#include "dock-avoid.h"
#include "virtual-wall.h"

#include "am-docking-utils.h"

#ifdef AM_DOCKING_METHOD

U8 dock_signals[IR_MAX_RECV];//need to get from core board
uint32_t dock_signal_get_time;

#define DOCK_SENSORS_DEBUG
#ifdef DOCK_SENSORS_DEBUG
enum {
	DEBUG_DOCK_SIGNAL = 0x1 << 0,
	DEBUG_DOCK_FILTER = 0x1 << 1,
};

//#define AM_DOCKING_ALL_NIGHT_LONG
#ifndef AM_DOCKING_ALL_NIGHT_LONG
static U8 debug_mask = DEBUG_DOCK_SIGNAL;
#else
static U8 debug_mask = DEBUG_DOCK_SIGNAL;
#endif
#define dprintf(level_mask, msg...)  if(debug_mask&level_mask)\
	printf(msg)
#else
#define dprintf(msg...) {}
#endif

BOOLEAN force_field(U8 chan)
{
	if(chan>=IR_LOCAL_MAX) return FALSE;
	return ((dock_signals[chan]&DOCK_CLOSE_BEACON) == DOCK_CLOSE_BEACON);
}

BOOLEAN buoy_left(U8 chan)
{
	if(chan>=IR_LOCAL_MAX) return FALSE;
	return ((dock_signals[chan]&LEFT_BEACON_BYTE) == LEFT_BEACON_BYTE);
}

BOOLEAN buoy_right(U8 chan)
{
	if(chan>=IR_LOCAL_MAX) return FALSE;
	return ((dock_signals[chan]&RIGHT_BEACON_BYTE) == RIGHT_BEACON_BYTE);
}

BOOLEAN buoy_center(U8 chan)
{
	if(chan>=IR_LOCAL_MAX) return FALSE;
	if(IR_LOCAL_IS_STEREO||(BOTH_BEACONS_BYTE==0x00))
	{
		return (recently_left_receiver[chan].current_state&&recently_right_receiver[chan].current_state);
	}
	else
	{
		return ((dock_signals[chan]&BOTH_BEACONS_BYTE) == BOTH_BEACONS_BYTE);
	}
}

BOOLEAN buoy_center_realTime(U8 chan)
{
	if(chan>=IR_LOCAL_MAX||(BOTH_BEACONS_BYTE==0x00)) return FALSE;
	return ((dock_signals[chan]&BOTH_BEACONS_BYTE) == BOTH_BEACONS_BYTE);
}

BOOLEAN buoy_all(U8 chan)
{
	if(chan>=IR_LOCAL_MAX) return FALSE;
	return (force_field(chan)||buoy_left(chan)||buoy_right(chan)||buoy_center(chan));
}

BOOLEAN buoy_all_realTime(U8 chan)
{
	if(chan>=IR_LOCAL_MAX) return FALSE;
	return (force_field(chan)||buoy_left(chan)||buoy_right(chan)||buoy_center_realTime(chan));
}

BOOLEAN check_near_dock(void)
{
	BOOLEAN is_near_dock = FALSE;
	for(u8 i = 0;i<6;i++)
		is_near_dock|=force_field(i);
	return is_near_dock;
}

BOOLEAN check_facing_dock(void)
{
	return FALSE;
}

BOOLEAN check_signal(void)
{
	BOOLEAN is_has_signal = FALSE;
	for(u8 i = 0;i<6;i++)
		is_has_signal|=buoy_all(i);
	return is_has_signal;
}

BOOLEAN all_ir_center(void)
{
	BOOLEAN is_center_signal = FALSE;
	for(u8 i = 0;i<6;i++)
		is_center_signal|=buoy_center(i);
	return is_center_signal;
}

BOOLEAN check_recently_left_0(void)
{
	return (buoy_left(0));
}
BOOLEAN check_recently_left_1(void)
{
	return (buoy_left(1));
}
BOOLEAN check_recently_left_2(void)
{
	return (buoy_left(2));
}
BOOLEAN check_recently_left_3(void)
{
	return (buoy_left(3));
}
BOOLEAN check_recently_left_4(void)
{
	return (buoy_left(4));
}
BOOLEAN check_recently_left_5(void)
{
	return (buoy_left(5));
}

BOOLEAN check_recently_right_0(void)
{
	return (buoy_right(0));
}
BOOLEAN check_recently_right_1(void)
{
	return (buoy_right(1));
}
BOOLEAN check_recently_right_2(void)
{
	return (buoy_right(2));
}
BOOLEAN check_recently_right_3(void)
{
	return (buoy_right(3));
}
BOOLEAN check_recently_right_4(void)
{
	return (buoy_right(4));
}
BOOLEAN check_recently_right_5(void)
{
	return (buoy_right(5));
}

BOOLEAN check_recently_all_mid(void)
{
	if(IR_LOCAL_IS_STEREO)
	{
		return (buoy_all_realTime(IR_LOCAL_MID_LEFT)||\
			buoy_all_realTime(IR_LOCAL_MID_RIGHT));
	}
	else
		return buoy_all_realTime(IR_LOCAL_CENTER_FOCUS);
}

BOOLEAN check_recently_center(void)
{
	BOOLEAN is_center_signal = FALSE;
	for(u8 i = 0;i<6;i++)
		is_center_signal|=buoy_center(i);
	return is_center_signal;
}

BOOLEAN check_recently_sender_left(void)
{
	BOOLEAN is_left_signal = FALSE;
	for(u8 i = 0;i<6;i++)
		is_left_signal|=buoy_left(i);
	return is_left_signal;
}

BOOLEAN check_recently_sender_right(void)
{
	BOOLEAN is_right_signal = FALSE;
	for(u8 i = 0;i<6;i++)
		is_right_signal|=buoy_right(i);
	return is_right_signal;
}

BOOLEAN check_recently_receiver_0(void)
{
    return buoy_all_realTime(0);
}
BOOLEAN check_recently_receiver_1(void)
{
    return buoy_all_realTime(1);
}
BOOLEAN check_recently_receiver_2(void)
{
    return buoy_all_realTime(2);
}
BOOLEAN check_recently_receiver_3(void)
{
    return buoy_all_realTime(3);
}
BOOLEAN check_recently_receiver_4(void)
{
    return buoy_all_realTime(4);
}
BOOLEAN check_recently_receiver_5(void)
{
    return buoy_all_realTime(5);
}

BOOLEAN check_recently_receiver_avoid_0(void)
{
    return force_field(0);
}
BOOLEAN check_recently_receiver_avoid_1(void)
{
    return force_field(1);
}
BOOLEAN check_recently_receiver_avoid_2(void)
{
    return force_field(2);
}
BOOLEAN check_recently_receiver_avoid_3(void)
{
    return force_field(3);
}
BOOLEAN check_recently_receiver_avoid_4(void)
{
    return force_field(4);
}
BOOLEAN check_recently_receiver_avoid_5(void)
{
    return force_field(5);
}

BOOLEAN check_docking_go_forward_right(void)
{
	if (IR_LOCAL_IS_STEREO&&\
		buoy_center(IR_LOCAL_MID_RIGHT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_left(void)
{
	if (IR_LOCAL_IS_STEREO&&\
		buoy_center(IR_LOCAL_MID_LEFT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_onlyright(void)
{
	if (IR_LOCAL_IS_STEREO&&\
		(buoy_right(IR_LOCAL_MID_RIGHT) &&\
		!buoy_left(IR_LOCAL_MID_RIGHT)))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_onlyleft(void)
{
	if (IR_LOCAL_IS_STEREO&&\
		(buoy_left(IR_LOCAL_MID_LEFT) &&\
		!buoy_right(IR_LOCAL_MID_LEFT)))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward(void)
{
	if(IR_LOCAL_IS_STEREO)
	{
		if ((recently_docking_go_forward_right.current_state && recently_docking_go_forward_left.current_state))
			return TRUE;
        else if((!recently_docking_go_forward_right.current_state && !recently_docking_go_forward_left.current_state)&&\
            (recently_docking_go_forward_onlyright.current_state && recently_docking_go_forward_onlyleft.current_state))
             return TRUE;
		else
			return FALSE;
	}
	else
	{
		if(buoy_center(IR_LOCAL_CENTER_FOCUS))
			return TRUE;
		else
			return FALSE;	
	}
}

BOOLEAN check_docking_left(void)
{
	if(IR_LOCAL_IS_STEREO)
	{
		if (buoy_right(IR_LOCAL_MID_LEFT))
			return TRUE;
		else
			return FALSE;
	}
	else
	{
		if((buoy_right(IR_LOCAL_CENTER_FOCUS)&&\
			!buoy_left(IR_LOCAL_CENTER_FOCUS))||\
			(IR_LOCAL_IS_BLEB&&\
			buoy_right(IR_LOCAL_BLEB) && !buoy_left(IR_LOCAL_BLEB)))
			return TRUE;
		else
			return FALSE;
	}
}

BOOLEAN check_docking_right(void)
{
	if(IR_LOCAL_IS_STEREO)
	{
		if (buoy_left(IR_LOCAL_MID_RIGHT))
			return TRUE;
		else
			return FALSE;
	}
	else
	{
		if((buoy_left(IR_LOCAL_CENTER_FOCUS)&&\
			!buoy_right(IR_LOCAL_CENTER_FOCUS))||\
			(IR_LOCAL_IS_BLEB&&\
			buoy_left(IR_LOCAL_BLEB) && !buoy_right(IR_LOCAL_BLEB)))
			return TRUE;
		else
			return FALSE;
	}
}

BOOLEAN check_docking_bump(void)
{
	if ((get_cliff_state() != 0) || (get_bump_state() != 0))
		return TRUE;
	else
		return FALSE;
}

void set_near_dock_context(BOOLEAN value)
{
	if(value)
	{
		//set_slip_high_throd(6);
		set_stasis_high_throd();
		turn_off_touch_bump();
		for(int i=0;i<3;i++)
                {
	          set_cliff_threshold((SENSOR_E)i, 20);
                }
		set_lighttouch_enable(1);
	}
	else
	{
		//set_slip_normal();
		set_stasis_normal();
		turn_on_touch_bump();
		reset_cliff_threshold();
		set_lighttouch_enable(0);
	}
	return;
}

Debouncer_Data recently_all_ir_signal = {
	.predicate = &check_signal,
	.trigger_on = 1,
	.trigger_off = 60000,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_all_ir_center = {
	.predicate = &all_ir_center,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};


Debouncer_Data recently_signal = {
	.predicate = &check_signal,
	.trigger_on = 1,
	.trigger_off = 2000,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_signal_short_term = {
	.predicate = &check_signal,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL,
};

Debouncer_Data recently_near_dock = {
	.predicate = &check_near_dock,
	.trigger_on = 1,
	.trigger_off = 2000,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL//&set_near_dock_context
};

Debouncer_Data recently_near_dock_2 = {
	.predicate = &check_near_dock,
	.trigger_on = 1,
	.trigger_off = 30,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_check_recently_all_mid = {
	.predicate = &check_recently_all_mid,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_center = {
	.predicate = &check_recently_center,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL,
};


Debouncer_Data recently_docking_go_forward_right = {
	.predicate = &check_docking_go_forward_right,
	.trigger_on = 1,
	.trigger_off = 5,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_left = {
	.predicate = &check_docking_go_forward_left,
	.trigger_on = 1,
	.trigger_off = 5,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_onlyright = {
	.predicate = &check_docking_go_forward_onlyright,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_onlyleft = {
	.predicate = &check_docking_go_forward_onlyleft,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward = {
	.predicate = &check_docking_go_forward,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_left = {
	.predicate = &check_docking_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_right = {
	.predicate = &check_docking_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_no_force_field = {
	.predicate = &check_docking_go_forward,
	.trigger_on = 1,
	.trigger_off = 200,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};


Debouncer_Data recently_bump = {
	.predicate = &check_docking_bump,
	.trigger_on = 1,
	.trigger_off = 30,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_receiver[6] = {
    {
        .predicate = &check_recently_receiver_0,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_1,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_2,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_3,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_4,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_5,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
};

Debouncer_Data recently_avoid_receiver[6] = {
    {
        .predicate = &check_recently_receiver_avoid_0,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_avoid_1,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_avoid_2,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_avoid_3,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_avoid_4,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_receiver_avoid_5,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
};

Debouncer_Data recently_sender[3] = {
    {
        .predicate = &check_recently_sender_left,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_center,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
    {
        .predicate = &check_recently_sender_right,
        .trigger_on = 1,
        .trigger_off = 30,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
};
Debouncer_Data recently_left_receiver[6] = {
	 {
        .predicate = &check_recently_left_0,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_left_1,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_left_2,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_left_3,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_left_4,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_left_5,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
};

Debouncer_Data recently_right_receiver[6] = {
	 {
        .predicate = &check_recently_right_0,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_right_1,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_right_2,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_right_3,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_right_4,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
     {
        .predicate = &check_recently_right_5,
        .trigger_on = 1,
        .trigger_off = 10,
        .on_count = 0,
        .off_count = 0,
        .current_state = FALSE,
        .set_dock_context = NULL,
    },
};


Debouncer_Data recently_facing_dock = {
	.predicate = &check_facing_dock,
	.trigger_on = 1,
	.trigger_off = 30,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

//#define FILTER_ALIASING_SIGNAL
#ifdef FILTER_ALIASING_SIGNAL

//#define FILTER_ALIASING_V1 // much stricter: <TRUE><TRUE><TRUE><FALSE>
//#define FILTER_ALIASING_V2 // filter <TRUE><TRUE><FALSE><TRUE><FALSE>, but at higher risk with mistake

extern float ir_receiver_distributions(int which);
extern float ir_receiver_angles_range(int which);

#define MAX_SIGNAL_HISTORY (6)
#define HALF_SIGNAL_HISTORY ((MAX_SIGNAL_HISTORY+1)/2)

typedef struct
{
   float start;
   float end;
} range_t;

typedef struct {
    AM_xy xy;
    range_t range;
} signal_pose_t;

struct {
    signal_pose_t poses[MAX_SIGNAL_HISTORY];
    BOOLEAN poses_valid[MAX_SIGNAL_HISTORY];
    signal_pose_t* last_pose;
    U8 index; // index to write
    U8 size;
    range_t range;
    BOOLEAN range_valid;
} signal_info;

// handle range in [0..180) only
static BOOLEAN start_in_range(float start, range_t* range) {
    return AM_math_angle_diff_f(start, range->start) >= 0 && AM_math_angle_diff_f(range->end, start) > 0;
}
static BOOLEAN end_in_range(float end, range_t* range) {
    return AM_math_angle_diff_f(end, range->start) > 0 && AM_math_angle_diff_f(range->end, end) >= 0;
}
static BOOLEAN range_is_intersect(range_t* r1, range_t* r2) {
    return start_in_range(r1->start, r2) || start_in_range(r2->start, r1);
}
static BOOLEAN range_intersect(range_t* r1, range_t* r2, range_t *out) {
    if (start_in_range(r1->start, r2)) {
        out->start = r1->start;
        out->end = end_in_range(r1->end, r2) ? r1->end : r2->end;
    } else if (start_in_range(r2->start, r1)) {
        out->start = r2->start;
        out->end = end_in_range(r2->end, r1) ? r2->end : r1->end;
    } else {
        return FALSE;
    }
    return TRUE;
}
static BOOLEAN range_intersect_by(range_t* r1, range_t* r2) {
    range_t out;
    BOOLEAN ret = range_intersect(r1, r2, &out);
    if (ret)
        *r1 = out;
    return ret;
}

void signal_pose_calc(int which, signal_pose_t* pose) {
    AM_Pose curr;
    robot_pos_get(&curr);
    float theta = AM_math_angle_add_f(curr.angle, ir_receiver_distributions(which));
    pose->xy = curr.xy;
    pose->range.start = AM_math_angle_diff_f(theta, ir_receiver_angles_range(which)/2);
    pose->range.end = AM_math_angle_add_f(theta, ir_receiver_angles_range(which)/2);
}

void signal_info_init() {
    signal_info.index = 0;
    signal_info.size = 0;
    memset(&signal_info.poses_valid, 0, sizeof(signal_info.poses_valid));
}

static int signal_info_get_valid_count() {
    int count = 0;
    for (int i = 0; i < MAX_SIGNAL_HISTORY; ++i) {
        if (signal_info.poses_valid[i])
            count++;
    }
    return count;
}
static int signal_info_revert_valid() {
    dprintf(DEBUG_DOCK_FILTER, "valid revert\r\n");
    for (int i = 0; i < MAX_SIGNAL_HISTORY; ++i) {
        signal_info.poses_valid[i] = !signal_info.poses_valid[i];
    }
}

void signal_info_update(signal_pose_t* pose) {
    if (signal_info.size == 0 || robot_xy_dist(&signal_info.last_pose->xy, &pose->xy) > PAD_WIDTH/1000.f) {
        signal_info.poses[0] = *pose;
        signal_info.last_pose = &signal_info.poses[0];
        signal_info.index = signal_info.size = 1;
        signal_info.range = pose->range;
        signal_info.range_valid = TRUE;
    } else {
        signal_info.poses[signal_info.index] = *pose;
        signal_info.last_pose = &signal_info.poses[signal_info.index];
        if (++signal_info.index >= MAX_SIGNAL_HISTORY)
            signal_info.index = 0;
        if (signal_info.size < MAX_SIGNAL_HISTORY)
            ++signal_info.size;

        range_t range = signal_info.last_pose->range;
        BOOLEAN valid = TRUE;
        int last = signal_info.index == 0 ? MAX_SIGNAL_HISTORY : signal_info.index-1; /* last pose */
        for (int i = 1; i < signal_info.size && valid; ++i) {
            last = last == 0 ? MAX_SIGNAL_HISTORY : last-1;
            valid = valid && range_intersect_by(&range, &signal_info.poses[last].range);
        }
        signal_info.range = range;
        signal_info.range_valid = valid;
    }
}

void signal_info_update_v2(signal_pose_t* pose) {
    if (signal_info.size == 0 || robot_xy_dist(&signal_info.last_pose->xy, &pose->xy) > PAD_WIDTH/1000.f) {
        signal_info.poses[0] = *pose;
        signal_info.poses_valid[0] = TRUE;
        signal_info.last_pose = &signal_info.poses[0];
        signal_info.index = signal_info.size = 1;
        signal_info.range = pose->range;
        signal_info.range_valid = TRUE;
    } else {
        signal_info.poses[signal_info.index] = *pose;
        signal_info.last_pose = &signal_info.poses[signal_info.index];
        signal_info.poses_valid[signal_info.index] = !signal_info.range_valid || range_is_intersect(&signal_info.range, &pose->range);
        if (++signal_info.index >= MAX_SIGNAL_HISTORY)
            signal_info.index = 0;
        if (signal_info.size < MAX_SIGNAL_HISTORY)
            ++signal_info.size;

        if (signal_info.size == MAX_SIGNAL_HISTORY && signal_info_get_valid_count() < HALF_SIGNAL_HISTORY) {
            signal_info_revert_valid();
        }

        range_t range;
        BOOLEAN first_range = TRUE;
        BOOLEAN valid = TRUE;
        int last = signal_info.index == 0 ? MAX_SIGNAL_HISTORY : signal_info.index-1; /* last pose */
        for (int i = 0; i < signal_info.size && valid; ++i) {
            if (signal_info.poses_valid[last]) {
                if (first_range) {
                    range = signal_info.poses[last].range;
                    first_range = FALSE;
                } else {
                    valid = valid && range_intersect_by(&range, &signal_info.poses[last].range);
                }
            }
            last = last == 0 ? MAX_SIGNAL_HISTORY : last-1;
        }
        signal_info.range = range;
        signal_info.range_valid = valid;
    }
}

#define R2D(r) ((int)(r*180/M_PI_F))

BOOLEAN signal_info_check_vaild(U8 index) {
    BOOLEAN ret = TRUE;;
    signal_pose_t pose;
    signal_pose_calc(index, &pose);
#ifdef FILTER_ALIASING_V1
    if (signal_info.size >= HALF_SIGNAL_HISTORY && signal_info.range_valid && !range_is_intersect(&signal_info.range, &pose.range)) {
        dprintf(DEBUG_DOCK_FILTER, "size=%d,range=(%d,%d)(%d,%d)\r\n", signal_info.size,
            R2D(signal_info.range.start), R2D(signal_info.range.end), R2D(pose.range.start), R2D(pose.range.end));
        ret = FALSE;
    }
    signal_info_update(&pose);
#endif
#ifdef FILTER_ALIASING_V2
    if (signal_info_get_valid_count() > HALF_SIGNAL_HISTORY && signal_info.range_valid && !range_is_intersect(&signal_info.range, &pose.range)) {
        dprintf(DEBUG_DOCK_FILTER, "size=%d,range=(%d,%d)(%d,%d)\r\n", signal_info.size,
            R2D(signal_info.range.start), R2D(signal_info.range.end), R2D(pose.range.start), R2D(pose.range.end));
        ret = FALSE;
    }
    signal_info_update_v2(&pose);
#endif
    return ret;
}

void aliasing_init_if_need() {
    static BOOLEAN inited = FALSE;
    if (!inited) {
        signal_info_init();
        inited = TRUE;
    }
}

U8 aliasing_filter(U8 signal, U8 index) {
    if (signal == 0 || (signal&0x0F) == 0)
        return signal;
    U32 t1 = timer_ms();
    aliasing_init_if_need();
    if (signal_info_check_vaild(index))
        return signal;
    U32 t2 = timer_ms();
    dprintf(DEBUG_DOCK_SIGNAL, "IR%d : %x filter,t=%d\r\n", index, signal, t2-t1);
    return 0;
}

#else
U8 aliasing_filter(U8 signal, U8 index) {
    return signal;
}
#endif

U8 robot_get_dock_signals(U8 index)
{
#ifdef IR_WIRELESS
	dock_signals[index] = get_wireless_rx_code();
#else
	dock_signals[index] = remote_ir_get((IR_REMOT_POSITION_E)index);
#endif

    dock_signals[index] = aliasing_filter(dock_signals[index], index);

	if (dock_signals[index] != 0)
	{
		//dprintf(DEBUG_DOCK_SIGNAL, "IR%d : %x \r\n", index, dock_signals[index]);
                
                dock_signal_get_time=timer_ms();
		dock_avoid_get_signals(index, dock_signals[index]);
		virtual_wall_get_signals(index, dock_signals[index]);
	}

	return dock_signals[index];
}

#endif // AM_DOCKING_METHOD