#ifndef ROBOT_TYPE_H
#define ROBOT_TYPE_H

#define ACCELERATION_MAX 1000
#define ACCELERATION_MIN 330
#define ACCELERATION_NON_EMERGENCY 600

typedef enum
{
    CLIFF_NONE           = 0,
    CLIFF_SIDE_LEFT      = (1 << 0),  
    CLIFF_FRONT_LEFT     = (1 << 1),  
    CLIFF_FRONT_CENTER   = (1 << 2),  
    CLIFF_FRONT_RIGHT    = (1 << 3),  
    CLIFF_SIDE_RIGHT     = (1 << 4)
} CliffState;

typedef enum
{
    BUMP_MASK_NONE = 0,                        
    BUMP_FRONT_LEFT    = (1 << 0), 
    BUMP_FRONT_RIGHT   = (1 << 1), 
    BUMP_FRONT_CENTER  = (1 << 2), 
    BUMP_LEFT          = (1 << 3),
    BUMP_RIGHT         = (1 << 4),
    BUMP_ALL           = 0x1F
} BumpState;

typedef enum {AM_DIR_NONE=-1, AM_RIGHT=0, AM_LEFT=1, AM_FORWARD=2, AM_BACKWARD=3} AM_LeftRight;


#endif