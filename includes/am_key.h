#ifndef __KEY_CONFIG_H__
#define __KEY_CONFIG_H__

#include "am_type.h"
#include "am_gpio.h"

#define TOTAL_KEY_NUM 5

/*change by luenwan*/

#define AM_IO_BUTTON_SPOT     -1
#define AM_IO_BUTTON_DOCK     -1
#define AM_IO_BUTTON_CLEAN    -1
#define AM_IO_BUTTON_DAY      -1
#define AM_IO_BUTTON_HOUR     -1
#define AM_IO_BUTTON_MIN      -1
#define AM_IO_BUTTON_CLOCK    -1

#define PRESS_NULL 0

typedef enum
{
	  KEY_NULL         = 0x00000000,
	  KEY_DOWN         = 0x00000001,
	  KEY_LONG         = 0x00000002,
	  KEY_HOLD         = 0x00000004,
	  KEY_SHORT_UP     = 0x00000008,
	  KEY_LONG_UP      = 0x00000010,
	  KEY_HOLD_UP      = 0x00000020,             
    KEY_CLEAN        = 0x00000100,
    KEY_SPOT         = 0x00000200,                   
    KEY_DOCK         = 0x00000400,
    KEY_CLOCK        = 0x00000800,
    KEY_SCHEDULE     = 0x00001000,
    KEY_POWER        = 0x00002000,
    KEY_LEFT         = 0x00004000,
    KEY_RIGHT        = 0x00008000, 
    KEY_FORWARD      = 0x00010000,
    KEY_STOP         = 0x00020000,
    KEY_BACKWARD     = 0x00040000,
    KEY_DIRT         = 0x00080000,
    KEY_WALLFLOW     = 0x00100000,
    KEY_SOS          = 0x00200000,
    KEY_WIFI         = 0X00400000,
    KEY_MODE         = 0X00800000,
    KEY_GRID         = 0X01000000,
    KEY_ADJUST       = 0X02000000,
    KEY_ADJUST1      = 0X04000000,
    KEY_RESERVED_MAX = 0x80000000
}key_val_e;

#endif  /* __KEY_CONFIG_H__ */

