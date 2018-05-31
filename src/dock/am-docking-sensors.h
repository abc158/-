//  Copyright (C) 2016-2020, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.
#ifndef _DOCKING_SENSORS_H_
#define _DOCKING_SENSORS_H_

extern BOOLEAN force_field(U8 chan);
extern BOOLEAN buoy_left(U8 chan);
extern BOOLEAN buoy_right(U8 chan);
extern BOOLEAN check_near_dock(void);
extern BOOLEAN check_docking_go_forward(void);
extern BOOLEAN check_docking_left(void);
extern U8 robot_get_dock_signals(U8 index);
extern BOOLEAN check_recently_all_mid(void);
extern void set_near_dock_context(BOOLEAN value);

extern Debouncer_Data recently_signal;
extern Debouncer_Data recently_signal_short_term;
extern Debouncer_Data recently_near_dock;
extern Debouncer_Data recently_near_dock_2;
extern Debouncer_Data recently_docking_left;
extern Debouncer_Data recently_docking_right;
extern Debouncer_Data recently_docking_go_forward_right;
extern Debouncer_Data recently_docking_go_forward_left;
extern Debouncer_Data recently_docking_go_forward_onlyright;
extern Debouncer_Data recently_docking_go_forward_onlyleft;
extern Debouncer_Data recently_docking_go_forward;
extern Debouncer_Data recently_force_field;
extern Debouncer_Data recently_no_force_field;
extern Debouncer_Data recently_check_recently_all_mid;
extern Debouncer_Data recently_center;
extern Debouncer_Data recently_all_ir_center;
extern Debouncer_Data recently_all_ir_signal;
extern Debouncer_Data recently_receiver[6];
extern Debouncer_Data recently_avoid_receiver[6];
extern Debouncer_Data recently_sender[3];
extern Debouncer_Data recently_facing_dock;
extern Debouncer_Data recently_left_receiver[6];
extern Debouncer_Data recently_right_receiver[6];

#endif //_DOCKING_NEW_H_
