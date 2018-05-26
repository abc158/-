//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------

#include "docking-new.h"
#include "sensor/sensor.h"
#include "virtual-wall.h"
#include "ui-ir.h"
#ifdef IR_WIRELESS
#include "dock_ir_signal.h"
#include "wireless/arf2496k.h"
#endif

#ifdef USE_VIRTUAL_WALL
#define VIRTUAL_WALL_SIGNAL_TIME_THRD 1200
#define VIRTUAL_WALL_SIGNAL_ACTIVE(time) (timer_elapsed(time) < VIRTUAL_WALL_SIGNAL_TIME_THRD)

static U32 see_virtual_wall_signal_time;

/* initialize a Always-On Virtual Wall pulse decoder */
int16_t aovw_decode_init(u16 instance)
{    
  return 0;
}

int16_t aovw_decode_ir(u16 instance, u16 ir_state)
{
  return -1;
}
#endif


void virtual_wall_get_signals(U8 index, U8 signal)
{
#ifdef USE_VIRTUAL_WALL
	dock_config_t *dock_config = NULL;

	dock_config = get_dock_config();

	if ((1 << index) & dock_config->aovw_chan)
	{
		if (signal == AOVW_BYTE)
		{
			see_virtual_wall_signal_time = timer_ms();
		}
	}
#endif
	return;
}

#ifdef USE_VIRTUAL_WALL
static U32 get_virtual_wall_signal_time(void)
{
	return see_virtual_wall_signal_time;
}
#endif

/* 正常清扫时，虚拟墙检测；
 */
BOOLEAN virtual_wall_active(void)
{
#ifdef USE_VIRTUAL_WALL
    return VIRTUAL_WALL_SIGNAL_ACTIVE(get_virtual_wall_signal_time());
#else
    return FALSE;
#endif
}



