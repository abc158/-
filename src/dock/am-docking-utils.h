//  Copyright (C) 2016-2020, Amicro, Inc.
//  permission of Amicro, Inc.  All rights reserved.

#ifndef _AM_DOCKING_UTILS_H_
#define _AM_DOCKING_UTILS_H_

// sdk <-> app
typedef BOOLEAN am_boolean;
typedef u8 am_uint8;
typedef s8 am_int8;
typedef u32 am_uint32;
typedef u16 am_uint16;
typedef s16 am_int16;
typedef S64 am_int64;
typedef AM_Pose AM_Pose2D;
#define localization_get_global_pos robot_pos_get


#define I(x) ((int)(x*1000)) // for printf float
#define R2D(r) ((int)(r*180/M_PI_F))
#define D2R(d) (float)(d*0.01745239f)


float AM_math_angle_diff_f(float angle1, float angle2);
float AM_math_angle_add_f(float angle1, float angle2);

/// api from sdk
void AM_Pose2D_transform_xy(const AM_Pose *pose, const AM_xy* xy, AM_xy *pose_xy);
void turn_in_place(am_real limit, int speed);
int64_t rate_gyro_get_angle_int(void);
void robot_to_world(const AM_Pose2D *cur_robot_pos, const AM_xy *sample_point, AM_xy *out);
void turn_circle(am_real limit, int speed);
void irmap_mutex_init();
void *irmap_mutex_lock();
void irmap_mutex_unlock(void *lock);

/// tip check
am_boolean am_tip_search_info(am_boolean stop_when_center);
void am_tip_get(void *info, int size);

/// irmap
#define IR_POSE_MAX_SIZE (100) // sync with sdk
#define IR_HYPOTHESES_EACH (16) // 

// x,y: -32768~32767 -> (-327.68, 327.67), step=0.01
// theta: -128~127 ->(-192,190.5)deg, step=1.5deg
typedef struct tag_IRPoint {
    am_int16 x, y;
    am_int8 theta;
    am_uint8 ir;
} IRPoint;
// ring buffer
// head == tail, empty
// tail+1 == head, full
typedef struct tag_IRTraj {
    am_uint8 head;
    am_uint8 tail;
    am_uint8 count;
    am_uint8 capacity;
    IRPoint points[IR_POSE_MAX_SIZE];
} IRTraj;

typedef struct tag_Hypotheses {
    am_uint8 count;
    am_uint8 capacity;
    AM_Pose2D models[IR_HYPOTHESES_EACH];
} Hypotheses;

void ir_map_init(IRTraj *traj);
void ir_map_add_traj_pose(IRTraj *traj, AM_Pose2D* pose, unsigned char ir, int which);
am_boolean ir_map_get_last_pose(IRTraj *traj, AM_Pose2D *pose);
am_boolean ir_map_get_last_certer_pose(IRTraj *traj, AM_Pose2D *pose);
void ir_map_clear_traj(IRTraj *traj);
int ir_map_traj_size(IRTraj *traj);
int ir_map_guess_dock_pose(IRTraj* traj, IRTraj* traj_copy, Hypotheses* hypotheses, AM_Pose2D* dock_pose, int* inlier, int* total);


/// pattern
typedef void (*pattern_get_target_t)(AM_xy *target);
typedef BOOLEAN (*pattern_gen_point_t)(u32 index, AM_xy *p);
typedef s16 (*pattern_max_vel_t)();

typedef int (*pattern_pose_init_t)(AM_Pose *pose);
typedef int (*pattern_cycle_squarewave_init_t)(am_real l, am_real w, am_real ww, BOOLEAN turn_right);
typedef int (*pattern_gb_init_t)(am_real r);
typedef int (*pattern_squarewave_init_t)(am_real l, am_real w, am_real ww);
typedef int (*pattern_circle_init_t)(int type, am_real r, BOOLEAN clockwise, BOOLEAN wf);
typedef void (*pattern_circle_strip_t)(AM_Pose* strip_pose);
typedef int (*pattern_safe_init_t)(float radius, int angle, float offset, BOOLEAN clockwise);


typedef struct {
    pattern_pose_init_t init;
    pattern_get_target_t get_target;
    pattern_gen_point_t  gen_point;
    pattern_max_vel_t    max_vel;
} pattern_pose_t;

typedef struct {
    pattern_cycle_squarewave_init_t init;
    pattern_get_target_t get_target;
    pattern_gen_point_t  gen_point;
    pattern_max_vel_t    max_vel;
} pattern_cycle_squarewave_t;

typedef struct {
    pattern_gb_init_t init;
    pattern_get_target_t get_target;
    pattern_gen_point_t  gen_point;
    pattern_max_vel_t    max_vel;
} pattern_gb_t;

typedef struct {
    pattern_squarewave_init_t init;
    pattern_get_target_t get_target;
    pattern_gen_point_t  gen_point;
    pattern_max_vel_t    max_vel;
} pattern_squarewave_t;

typedef struct {
    pattern_circle_init_t init;
    pattern_get_target_t get_target;
    pattern_gen_point_t  gen_point;
    pattern_max_vel_t    max_vel;
    pattern_circle_strip_t strip;
} pattern_circle_t;

typedef struct {
    pattern_safe_init_t init;
    pattern_get_target_t get_target;
    pattern_gen_point_t  gen_point;
    pattern_max_vel_t    max_vel;
} pattern_safe_t;

extern const pattern_pose_t pattern_pose;
extern const pattern_cycle_squarewave_t pattern_cycle_squarewave;
extern const pattern_gb_t pattern_gb;
extern const pattern_squarewave_t pattern_squarewave;
extern const pattern_circle_t pattern_circle;
extern const pattern_safe_t pattern_safe;


s16 pattern_default_max_vel();

int pattern_pose_init(AM_Pose *pose);
void pattern_pose_get_target(AM_xy *target);
BOOLEAN pattern_pose_gen_point(u32 index, AM_xy *p);

int pattern_cycle_squarewave_init(am_real l, am_real w, am_real ww, BOOLEAN turn_right);
void pattern_cycle_squarewave_get_target(AM_xy *target);
BOOLEAN pattern_cycle_squarewave_gen_point(u32 index, AM_xy *p);
s16 pattern_cycle_squarewave_set_max_vel();

int pattern_gb_init(am_real r);
void pattern_gb_get_target(AM_xy *target);
BOOLEAN pattern_gb_gen_point(u32 index, AM_xy *p);

int pattern_squarewave_init(am_real l, am_real w, am_real ww);
BOOLEAN pattern_squarewave_gen_point(u32 index, AM_xy *p);
void pattern_squarewave_get_target(AM_xy *target);
s16 pattern_squarewave_set_max_vel();

void pattern_circle_get_target(AM_xy *target);
BOOLEAN pattern_circle_gen_point(u32 index, AM_xy *p);
int pattern_circle_init(int type, am_real r, BOOLEAN clockwise, BOOLEAN wf);
s16 pattern_circle_set_max_vel();
void pattern_circle_strip(AM_Pose* strip_pose);



#endif // _AM_DOCKING_UTILS_H_

