#include "am-docking-config.h"
#include "am-docking-utils.h"
#include "syscall_api.h"

#define NELEMS(arr) (sizeof(arr)/sizeof(arr[0]))

typedef struct {
    u8 beacon;
    BOOLEAN is_center;
    BOOLEAN is_avoid;
} beacon_info_t;
static beacon_info_t _beacon_infos[] = {
    { 0xbc, TRUE, FALSE },
    { 0xb1, FALSE, TRUE },
    { 0xb4, FALSE, FALSE },
    { 0xb8, FALSE, FALSE },

};

static u8 dock_center_index[2] = {
    0, 1, 
};

static u8 dock_bleb_index = 0;
static u16 dock_avoid_radius = 400;//mm

static const u8 _ir_map[20 * 20] = {
    0x00, 0x00, 0x00, 0x02, 0x02, 0x0a, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x06, 0x06, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x0a, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x06, 0x06, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0a, 0x0a, 0x0f, 0x0f, 0x0f, 0x0f, 0x06, 0x06, 0x06, 0x02, 0x02, 0x02, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x08, 0x08, 0x0f, 0x0d, 0x0f, 0x06, 0x06, 0x06, 0x06, 0x00, 0x02, 0x02, 0x02, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x0d, 0x0d, 0x0f, 0x06, 0x06, 0x04, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x00, 0x02, 0x00, 0x02, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x0d, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0c, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x0c, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x0c, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x0c, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0c, 0x0c, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x08, 0x0c, 0x0c, 0x0c, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x08, 0x0c, 0x0c, 0x0c, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 
    0x00, 0x08, 0x08, 0x00, 0x08, 0x0c, 0x0d, 0x0d, 0x0d, 0x0d, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 

};

static const int _ir_distributions[IR_RECEIVER_COUNT] = {
    5, -5, 50, -50, 120, -120, 
};

static const int _ir_receiver_ranges[IR_RECEIVER_COUNT] = {
    20, 20, 80, 80, 80, 80, 
};



u8 ir_map(int index) {
    return _ir_map[index];
}

u8 ir_sender_count() {
    return IR_SENDER_COUNT;
}

u8 ir_receiver_count() {
    return IR_RECEIVER_COUNT;
}

float ir_receiver_distributions(int which) {
    return _ir_distributions[which] * M_PI_F / 180;
}

float ir_receiver_angles_range(int which) {
    return _ir_receiver_ranges[which] * M_PI_F / 180;
}

BOOLEAN ir_local_center_is_stereo()
{
	return DOCK_IS_STEREO;
}

u8 get_ir_local_center_index(u8 which)
{
	return dock_center_index[which];
}

BOOLEAN ir_local_is_bleb()
{
	return DOCK_IS_BLEB;
}

u8 get_ir_local_bleb_index()
{
	return dock_bleb_index;
}

u16 get_ir_dock_avoid_radius()
{
	return dock_avoid_radius;
}

static BOOLEAN match_beacon(u8 signal, u8 beacon) {
    return (beacon != 0) && ((signal & beacon) == beacon);
}

u8 get_ir_with_beacons(u8 beacon) {
    u8 ir = 0;
    for (int i = 0; i < NELEMS(_beacon_infos); ++i) {
        beacon_info_t* info = _beacon_infos + i;
        if (match_beacon(beacon, info->beacon)) {
            ir |= (1 << i);
        }
    }
    return ir;
}
static u8 get_beacons_with_ir(u8 ir) {
    return 0;
}

BOOLEAN ir_in_center(u8 ir) {
    for (int i = 0; i < NELEMS(_beacon_infos); ++i) {
        beacon_info_t* info = _beacon_infos + i;
        if ((ir & (1 << i)) && info->is_center)
            return TRUE;
    }
    return FALSE;
}

BOOLEAN ir_no_signal(u8 ir) {
    return ir == 0;
}

BOOLEAN ir_match(u8 ir_in_map, u8 ir_receive) {
    return ir_in_map & ir_receive;
}


BOOLEAN ir_good_distribution() {
    return false;
}
//for dockeasy signal scan
#ifdef DOCK_EASY_TEST

#include "lib.h"
#define DOCK_EASY_IRMAP_SCAN_TEST 0x01
#define DOCK_EASY_GB_SIGNAL_SCAN_TEST 0x02
#define DOCK_EASY_CENTER_SIGNAL_SCAN_TEST 0x03

static int _am_dock_easy_mode = 0;
static u8 last_system_state = 0;  
static sys_state_info sys_state_info_p;

#define UART_CTRL_CMD_DATA_LEN 6
static const u8 uart_ctrl_cmd_reset[] = {0xAA, 0x03, 0x02, 0x10, 0x02, 0x14};
static const u8 uart_ctrl_cmd_spot[]  = {0xAA, 0x03, 0x02, 0x22, 0x01, 0x25};
static const u8 uart_ctrl_cmd_dock[]  = {0xAA, 0x03, 0x02, 0x22, 0x03, 0x27};
static const u8 uart_ctrl_cmd_clean[] = {0xAA, 0x03, 0x02, 0x22, 0x02, 0x26};

static u8 uart_rx_data[32];
static u8 uart_rx_len = 0;

static int _memcmp(const u8* buf1, u8* buf2, int len)
{
   int i, ret = 0;
   for(i=0; i<len; i++)
   {
     if(buf1[i] < buf2[i]) {
       ret = -1;
       break;
     }
     if(buf1[i] > buf2[i]) {
       ret = 1;
       break;
     }
   }
   return ret;
}

void dock_easy_uart_rx_handle(int data)
{  
  u8 i;
  if (uart_rx_len >= sizeof(uart_rx_data)) {
     uart_rx_len = 0;     
  }
  uart_rx_data[uart_rx_len] = (u8)(data & 0x0FF);
  uart_rx_len++;
  for (i=0; i <= uart_rx_len - UART_CTRL_CMD_DATA_LEN; i++) {
    if (uart_rx_data[i] == 0xAA) {
      if (_memcmp(uart_ctrl_cmd_reset, &uart_rx_data[i], UART_CTRL_CMD_DATA_LEN) == 0) {
          system_reboot();
      } 
      else if (_memcmp(uart_ctrl_cmd_spot, &uart_rx_data[i], UART_CTRL_CMD_DATA_LEN) == 0) {
          printf("run DOCK_EASY_IRMAP_SCAN_TEST\r\n");
          _am_dock_easy_mode = DOCK_EASY_IRMAP_SCAN_TEST;
		  uart_rx_len = 0;
          act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);          
		  break;
      } 
      else if (_memcmp(uart_ctrl_cmd_dock, &uart_rx_data[i], UART_CTRL_CMD_DATA_LEN) == 0) {
          printf("run DOCK_EASY_GB_SIGNAL_SCAN_TEST\r\n");
          _am_dock_easy_mode = DOCK_EASY_GB_SIGNAL_SCAN_TEST;
		  uart_rx_len = 0;
          act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);          
		  break;
      } 
      else if (_memcmp(uart_ctrl_cmd_clean, &uart_rx_data[i], UART_CTRL_CMD_DATA_LEN) == 0) {
          printf("run DOCK_EASY_CENTER_SIGNAL_SCAN_TEST\r\n");
          _am_dock_easy_mode = DOCK_EASY_CENTER_SIGNAL_SCAN_TEST;
          uart_rx_len = 0;
          act_command_q(CMD_DOCK, CMD_RUN, NULL, 0);          
		  break;
      } 
    }
  }
}

void dockeasy_print_signal(int index, int signal)
{
    AM_Pose robot_pose;
    robot_pos_get(&robot_pose);
    int x = (int)((robot_pose.xy.x*1000)+3000);
    int y = (int)((robot_pose.xy.y*1000)+3000);
    int angle = (int)((robot_pose.angle/M_PI_F+2)*180);  
    printf("(%d,%d,%d)->(%04X)\r\n",x,y,angle,((index <<12) | (signal & 0x0FFF)) | (0x01<<8)); //only strong signal
}

void dockeasy_check_finish(void)
{
  if (_am_dock_easy_mode != 0) {
    sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
    if (last_system_state != sys_state_info_p.robot_state)
    {
      if (last_system_state == ROBOT_STATE_DOCK)      {
         _am_dock_easy_mode = 0;     
         printf("(%d,%d,%d)->(%04X)\r\n",0,0,0,0xFFFF);
         printf("dock easy scan pattern finish!\r\n");
      }
      last_system_state = sys_state_info_p.robot_state;
    }
  }
}

int am_get_dock_easy(void)
{
    return _am_dock_easy_mode;
}

//not defined DOCK_EASY_TEST
#else 

int am_get_dock_easy(void)
{
    return 0;
}

#endif

//#define DOCK_EASY_LT_HOLD_TIME
#ifndef DOCK_EASY_LT_HOLD_TIME
 u8 set_lt_hold_on_time(u32 time)
{
	return 0;
}
#else
extern u8 set_lt_hold_on_time(u32 time);
#endif
extern u8 get_dock_kidnap_state();
extern void set_dock_kidnap_state(u8 s);

enum {
    AM_OPS_IR_SENDER_COUNT = 0,
    AM_OPS_IR_RECEIVER_COUNT,
    AM_OPS_GET_IR_WITH_BEACONS,
    AM_OPS_GET_BEACONS_WITH_IR,
    AM_OPS_IR_LOCAL_IS_BLEB,
    AM_OPS_GET_IR_LOCAL_BLEB_INDEX,
    AM_OPS_SET_LT_HOLD_ON_TIME,
    AM_OPS_IR_GOOD_DISTRIBUTION,
    AM_OPS_GET_IR_DOCK_AVOID_RADIUS,
    AM_OPS_GET_DOCK_KIDNAP_STATE,
    AM_OPS_SET_DOCK_KIDNAP_STATE,
    AM_OPS_PATTERN_CYCLE_SQUAREWAVE_INIT,
    AM_OPS_PATTERN_CYCLE_SQUAREWAVE_GET_TARGET,
    AM_OPS_PATTERN_CYCLE_SQUAREWAVE_GEN_POINT,
    AM_OPS_PATTERN_CYCLE_SQUAREWAVE_MAX_VEL,
    AM_OPS_PATTERN_GB_INIT,
    AM_OPS_PATTERN_GB_GET_TARGET,
    AM_OPS_PATTERN_GB_GEN_POINT,
    AM_OPS_PATTERN_GB_MAX_VEL,
    AM_OPS_PATTERN_SQUAREWAVE_INIT,
    AM_OPS_PATTERN_SQUAREWAVE_GET_TARGET,
    AM_OPS_PATTERN_SQUAREWAVE_GEN_POINT,
    AM_OPS_PATTERN_SQUAREWAVE_MAX_VEL,
    AM_OPS_PATTERN_CIRCLE_INIT,
    AM_OPS_PATTERN_CIRCLE_GET_TARGET,
    AM_OPS_PATTERN_CIRCLE_GEN_POINT,
    AM_OPS_PATTERN_CIRCLE_MAX_VEL,
    AM_OPS_PATTERN_CIRCLE_STRIP,
    AM_OPS_AM_TIP_SEARCH_INFO,
    AM_OPS_AM_TIP_GET,
    AM_OPS_AM_GET_DOCK_EASY,
    AM_OPS_IR_MAP_INIT,
    AM_OPS_IR_MAP_ADD_TRAJ_POSE,
    AM_OPS_IR_MAP_GET_LAST_POSE,
    AM_OPS_IR_MAP_GET_LAST_CERTER_POSE,
    AM_OPS_IR_MAP_CLEAR_TRAJ,
    AM_OPS_IR_MAP_TRAJ_SIZE,
    AM_OPS_IR_MAP_GUESS_DOCK_POSE,
    AM_OPS_MAX,
};

static void* const _ops[AM_OPS_MAX] = {
    (void*)ir_sender_count,
    (void*)ir_receiver_count,
    (void*)get_ir_with_beacons,
    (void*)get_beacons_with_ir,
    (void*)ir_local_is_bleb,
    (void*)get_ir_local_bleb_index,
    (void*)set_lt_hold_on_time,
    (void*)ir_good_distribution,
    (void*)get_ir_dock_avoid_radius,
    (void*)get_dock_kidnap_state,
    (void*)set_dock_kidnap_state,
    (void*)pattern_cycle_squarewave_init,
    (void*)pattern_cycle_squarewave_get_target,
    (void*)pattern_cycle_squarewave_gen_point,
    (void*)pattern_cycle_squarewave_set_max_vel,
    (void*)pattern_gb_init,
    (void*)pattern_gb_get_target,
    (void*)pattern_gb_gen_point,
    (void*)pattern_default_max_vel,
    (void*)pattern_squarewave_init,
    (void*)pattern_squarewave_get_target,
    (void*)pattern_squarewave_gen_point,
    (void*)pattern_squarewave_set_max_vel,
    (void*)pattern_circle_init,
    (void*)pattern_circle_get_target,
    (void*)pattern_circle_gen_point,
    (void*)pattern_circle_set_max_vel,
    (void*)pattern_circle_strip,
    (void*)am_tip_search_info,
    (void*)am_tip_get,
    (void*)am_get_dock_easy,
    (void*)ir_map_init,
    (void*)ir_map_add_traj_pose,
    (void*)ir_map_get_last_pose,
    (void*)ir_map_get_last_certer_pose,
    (void*)ir_map_clear_traj,
    (void*)ir_map_traj_size,
    (void*)ir_map_guess_dock_pose,
};

void *am_docking_ops(int index) {
    return _ops[index];
}

void setup_ir_config() {
    register_ir_ops((void*)am_docking_ops);
}
