
#ifndef __DEFTYPE_SIMSWEEP_H__
#define __DEFTYPE_SIMSWEEP_H__

#include "am_date_base.h"

typedef enum
{
    REMOTE_FORWARD=0x0,
    REMOTE_LEFT,
    REMOTE_RIGHT,
	REMOTE_BACK,
	REMOTE_IDLE,
}REMOTE_STATE_E;
//*************************
extern void send_base_info(void);
extern void send_robot_state_to_wifi(void);
extern void parser_wifi_selftest_result(u8 *data_buf);
extern void parser_wifi_state(U8 *data_buf);
extern void send_map_data_to_wifi(void);
extern void parser_control_cmd(U8 *data_buf);
extern void send_config_network_cmd(void);
extern void send_wifi_song_mute(void);
extern void send_wifi_song_unmute(void);
extern void clear_map_index(void);
extern void Resend_map_data(U8 *data_buf);
void ui_put_map_point_info(uint16_t x, uint16_t y, uint8_t type, uint16_t direction);
extern void enter_wifi_selftest_state(void);
extern u8 get_wifi_selftest_result(void);
extern void check_wifi_state(void);

#define MAP_POINT_BUFFER_SIZE 16
#define MAP_FRAME_RETRY_COUNT 5

#define HISTORY_MAP_POINT_SIZE 50

typedef struct
{
    map_point_t points[MAP_POINT_BUFFER_SIZE];
    uint16_t direction;
    uint8_t count;
} real_map_points_t;


typedef struct 
{
    U16 x;
    U16 y;
	U16 his_index;
    U8  type;
}history_map_point_t;


/*

typedef	union
{
    struct
	{	
      U8 linked  :1;  // =1 已经连接上网络  =0 未连接上网络
      U8 linking :1;  // =1 进入连接状态    =0 退出连接状态
      U8 rev1  :6;    
    }bits;
    U8 byte;
} WIFI_STATE_BYTE;
*/


#endif


































