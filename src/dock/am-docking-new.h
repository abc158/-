#ifndef _DOCKING_NEW_H_
#define _DOCKING_NEW_H_
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "am_type.h"
#include "am_robot_type.h"
#include "syscall_api.h"
#include "docking-core.h"
#include "am-docking-config.h"

//#define  IR_WIRELESS
//#define USE_VIRTUAL_WALL

#define  DOCKINT_RANDOM_THRESHOLD  (2 * 6000)
#define  DOCKING_MAX_FAILURE_COUNT 5
#define  DOCKING_MAX_AVOID_OBSTACLE_COUNT 2

#define DOCK_CLOSE_BEACON DOCK_FORCE_FIELD
#define LEFT_BEACON_BYTE DOCK_BUOY_GREEN
#define RIGHT_BEACON_BYTE DOCK_BUOY_RED
#define BOTH_BEACONS_BYTE DOCK_BUOY_BOTH

#define AOVW_BYTE 0xFF

#define IS_BEACONS_BYTE(b)  ( (( b & (DOCK_CLOSE_BEACON&0xF0) ) == (DOCK_CLOSE_BEACON&0xF0))\
							&& ( b & 0x0F) )

#define IR_LOCAL_IS_STEREO DOCK_IS_STEREO
#define IR_LOCAL_IS_BLEB DOCK_IS_BLEB
#define IR_LOCAL_MID_LEFT  get_ir_local_center_index(0)
#define IR_LOCAL_MID_RIGHT get_ir_local_center_index(1)
#define IR_LOCAL_CENTER_FOCUS get_ir_local_center_index(0)
#define IR_LOCAL_BLEB get_ir_local_bleb_index()
#define IR_LOCAL_MAX IR_RECEIVER_COUNT
typedef enum
{
	DOCKING_SUCESS = 0XAA,
	DOCKING_FAIL = 0X88,	
    DOCKING_FAIL_ATTEMPT_MAX,    
    DOCKING_FAIL_FORCE_FIEDL,    
    DOCKING_FAIL_NO_CONDITION,    
    DOCKING_FAIL_NO_SIGNAL,
    DOCKING_FAIL_OBSTACLE,
    DOCKING_CLIFF = 0XA8,
} DockResult;

typedef enum
{
  DOCKING_BOUNCE,//0
  DOCKING_SUCCESS,
  DOCKING_FORCE_FIELD,
  DOCKING_AVOID_OBSTACLE,
  DOCKING_LINE_BOUNCE,
  DOCKING_GO_FORWARD, //5
  DOCKING_LEFT,
  DOCKING_RIGHT,
  DOCKING_CORRECT,
  DOCKING_RESERVE9,
  DOCKING_RESERVE10,
  DOCKING_RESERVE11,
  DOCKING_RESERVE12, 
  DOCKING_RESERVE13,
  DOCKING_RESERVE14,
  DOCKING_RESERVE15,
  DOCKING_RESERVE16,
  DOCKING_RESERVE17,
  DOCKING_RESERVE18,
  DOCKING_RESERVE19,
  DOCKING_RESERVE20,
  DOCKING_RESERVE21,
  DOCKING_RESERVE22,
  DOCKING_RESERVE23,
  DOCKING_RESERVE24,
  DOCKING_RESERVE25,
  DOCKING_RESERVE26,
  DOCKING_RESERVE27,
  DOCKING_RESERVE28,
  DOCKING_RESERVE29,
  DOCKING_RESERVE30,
  DOCKING_WHERE
}Bhavior_Id;

#define IR_MAX_RECV IR_LOCAL_MAX

typedef struct tagDockingState
{
  U32 state_cnt;
  BOOLEAN dock_finished;
  U8 dock_failured;
  U8 avoid_obstacle_count;
  U32 random_behavior_count;
  U8  dock_failure_count;
  U16 dock_angle;
  BOOLEAN near_dock;
  BOOLEAN dock_start_flag;
#ifdef IR_WIRELESS
  U8 wireless_data;
#endif
} DockingState;

extern U8 dock_signals[IR_MAX_RECV];

#ifdef IR_WIRELESS
extern void dock_wireless_rx_code_get(u8 chan, u8 val);
extern void clear_dock_wireless_rx_code(void);
extern U8 get_wireless_rx_code(void);
#endif

extern dock_config_t* dock_new_init(void);
extern void dock_new_start(void);
BOOLEAN dock_new_end(U8 *uTerm);
extern void set_dock_new_end(void);
extern dock_config_t *get_dock_config(void);
void set_dock_leave_dockagain(U8 state);

#endif //_DOCKING_NEW_H_
