#ifndef _DOCKING_NEW_H_
#define _DOCKING_NEW_H_
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "am_type.h"
#include "am_robot_type.h"
#include "syscall_api.h"
#include "docking-core.h"

//#define  IR_WIRELESS
//#define USE_VIRTUAL_WALL

#define  DOCKING_SUCESS 0XAA
#define  DOCKING_FAIL 0X88
#define  DOCKING_CLIFF 0XA8
#define  DOCKINT_RANDOM_THRESHOLD  (2 * 6000)

#define DOCK_CLOSE_BEACON 0xB1
#define LEFT_BEACON_BYTE 0xB4
#define RIGHT_BEACON_BYTE 0xB8
#define BOTH_BEACONS_BYTE 0xBC

#define AOVW_BYTE 0xFF

#define IS_BEACONS_BYTE(b)  ( (( b & (DOCK_CLOSE_BEACON&0xF0) ) == (DOCK_CLOSE_BEACON&0xF0))\
							&& ( b & 0x0F) )
typedef enum
{
  IR_LOCAL_MID_LEFT,
  IR_LOCAL_MID_RIGHT,
  IR_LOCAL_LEFT,
  IR_LOCAL_RIGHT,
  IR_LOCAL_BACK_LEFT,
  IR_LOCAL_BACK_RIGHT,
  IR_LOCAL_MAX
}IR_local_Index;
typedef enum
{
  DOCKING_BOUNCE,
  DOCKING_SUCCESS,
  DOCKING_FORCE_FIELD,
  DOCKING_AVOID_OBSTACLE,
  DOCKING_DOCK_AHEAD,
  DOCKING_LEFT_RIGHT,
  DOCKING_FIND_BUOY,
  DOCKING_LINE_BOUNCE,
  DOCKING_GO_FORWARD,
  DOCKING_LEFT,
  DOCKING_RIGHT,
  DOCKING_CORRECT,
  DOCKING_ONLY_FORCE_FIELD,
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
  DOCKING_LINE
}Bhavior_Id;

#define IR_MAX_RECV IR_LOCAL_MAX

typedef struct tagDockingState
{
  U32 state_cnt;
  BOOLEAN dock_finished;
  U32 random_behavior_count;
  S64 random_current_count;
  U16 dock_angle;
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