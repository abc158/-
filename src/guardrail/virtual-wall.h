#ifndef _VIRTUAL_WALL_H_
#define _VIRTUAL_WALL_H_
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "am_type.h"
#include "am_robot_type.h"
#include "syscall_api.h"

extern void virtual_wall_get_signals(U8 index, U8 signal);
extern BOOLEAN virtual_wall_active(void);

#endif //_DOCKING_NEW_H_