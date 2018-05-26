#ifndef _DOCK_AVOID_H_
#define _DOCK_AVOID_H_
#include "stdio.h"
#include "stdlib.h"
//#include "string.h"
#include "am_type.h"
#include "am_robot_type.h"
#include "syscall_api.h"

typedef struct
{
	U32 see_avoid_signal_time;
	U32 binocular_see_avoid_time;
	U32 left_see_avoid_time;
	U32 right_see_avoid_time;
} dock_avoid_data_t;

extern void dock_avoid_get_signals(U8 index, U8 signal);
extern BOOLEAN dock_avoid_active(BOOLEAN min);
extern U8 dock_avoid_signal_quality(void);
extern U8 dock_binocular_avoid_signal_quality(void);
extern U32 get_left_avoid_signal_time(void);
extern U32 get_right_avoid_signal_time(void);
extern U32 get_binocular_avoid_signal_time(void);

#endif //_DOCKING_NEW_H_