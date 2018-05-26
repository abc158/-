#ifndef __WALL_FOLLOW_COMMON_H__
#define __WALL_FOLLOW_COMMON_H__


#include <am_type.h>
#include <am_robot_type.h>


typedef enum
{
  WF_STAT_STOP=0,
  WF_STAT_TURN=1,
  WF_STAT_FOLLOW=2,
  WF_STAT_BACK=3,
  WF_STAT_STOPING = 4
}Wf_State_t;

typedef struct WfPar_struct
{
  AM_LeftRight contact_side;
  u8  robot_bump;    //0:none, 1:left, 2:right, 3:front, 4:back
  u8  cliff_bump;    //0:none, 1:cliff valid
  BOOLEAN physic_bump;  //true: physic bump, false not physic bump
  u8  dock_sig;
  BOOLEAN wf_following_sig;        //indicate that the robot is engage to wall
  s16 left_speed_mmps;         //set the speed
  s16 right_speed_mmps;        //set the speed
  Wf_State_t wf_run_state;
  BOOLEAN wf_is_trap;
}WfParames_t;


int wall_follow_start(AM_LeftRight contact_side, WfParames_t* wfp_p);
void wall_follow_stop(WfParames_t* wfp_p);
int wall_follow_callback(WfParames_t* wfp_p);

#endif
