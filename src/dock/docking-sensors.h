//  Copyright (C) 2016-2020, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.
#ifndef _DOCKING_SENSORS_H_
#define _DOCKING_SENSORS_H_

extern BOOLEAN force_field(IR_local_Index chan);
extern BOOLEAN buoy_left(IR_local_Index chan);
extern BOOLEAN buoy_right(IR_local_Index chan);
extern BOOLEAN check_near_dock(void);
extern u8 check_center_focus_beacon(void);
extern u8 check_right_beacon(void);
extern u8 check_left_beacon(void);
extern u8 check_back_right_beacon(void);
extern u8 check_back_left_beacon(void);
extern BOOLEAN check_docking_go_forward(void);
extern BOOLEAN check_docking_left(void);
extern U8 robot_get_dock_signals(U8 index);

extern BOOLEAN check_recently_right_centerright(void);
extern BOOLEAN check_recently_left_centerright(void);
extern BOOLEAN check_recently_left_centerleft(void);
extern BOOLEAN check_recently_right_centerleft(void);
extern BOOLEAN check_recently_right_right(void);
extern BOOLEAN check_recently_left_left(void);
extern BOOLEAN check_recently_right_backright(void);
extern BOOLEAN check_recently_left_backleft(void);



extern Debouncer_Data recently_near_dock;
extern Debouncer_Data recently_near_dock_1;
extern Debouncer_Data recently_docking_left;
extern Debouncer_Data recently_docking_right;
extern Debouncer_Data recently_left_left;
extern Debouncer_Data recently_right_right;
extern Debouncer_Data recently_left_backleft;
extern Debouncer_Data recently_right_backright;
extern Debouncer_Data recently_right_backleft;
extern Debouncer_Data recently_left_backright;
extern Debouncer_Data recently_center_left_focus;
extern Debouncer_Data recently_center_right_focus;
extern Debouncer_Data recently_left_right;
extern Debouncer_Data recently_right_left;
extern Debouncer_Data recently_bump;
extern Debouncer_Data recently_right_centerright;
extern Debouncer_Data recently_right_centerleft;
extern Debouncer_Data recently_left_centerleft;
extern Debouncer_Data recently_left_centerright;
extern Debouncer_Data recently_docking_left_midright;
extern Debouncer_Data recently_docking_right_midright;
extern Debouncer_Data recently_docking_left_midleft;
extern Debouncer_Data recently_docking_right_midleft;
extern Debouncer_Data recently_force_field_right;
extern Debouncer_Data recently_force_field_left;
extern Debouncer_Data recently_force_field_midright;
extern Debouncer_Data recently_force_field_midleft;
extern Debouncer_Data recently_force_field_centerright;
extern Debouncer_Data recently_force_field_centerleft;


#endif //_DOCKING_NEW_H_
