#ifndef __TEST_CMD_HANDLE_H
#define __TEST_CMD_HANDLE_H
#include "ui-typedef.h"
#include "am_robot_type.h"

#define UI_TEST_ITEM_MAX 15

extern void handle_test_cmd(U32 cmd);
extern int get_test_result(void);
extern void self_test_routine(void);

typedef enum
{
    WHEEL_DROP_STATE_NONE  = 0x00,
    WHEEL_DROP_STATE_LEFT  = 0x01,
    WHEEL_DROP_STATE_RIGHT = 0x02
} WheelDropState;

typedef enum
{
  WHEEL_LEFT  = 0,
  WHEEL_RIGHT = 1,
  MAIN_BRUSH  = 2,
  SIDE_BRUSH  = 3,
  SUCTION     = 4,
  ALL_TEST    = 5,
}MOTOR_E;

WheelDropState robot_wheel_drop_mask(void);
BumpState robot_bump_mask(void);
#endif /* __TEST_CMD_HANDLE_H */