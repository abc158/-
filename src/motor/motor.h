#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

typedef struct
{
    int32_t  left_ticks;
    uint32_t left_abs_ticks;
    int32_t  right_ticks;
    uint32_t right_abs_ticks;    
} motor_encode_t;


void robot_motor_init(void);

#endif