
#ifndef __DEFTYPE_SIMSWEEP_H__
#define __DEFTYPE_SIMSWEEP_H__


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
extern void send_wifi_test_result(void);
extern void parser_wifi_state(U8 *data_buf);
extern void send_map_data_to_wifi(void);
extern void parser_control_cmd(U8 *data_buf);
extern void send_config_network_cmd(void);
extern void send_wifi_song_mute(void);
extern void send_wifi_song_unmute(void);
extern void clear_map_index(void);
extern void parser_map_info(U8 *data_buf);
void ui_put_map_point_info(uint16_t x, uint16_t y, uint8_t type, uint16_t direction);

#define MAP_POINT_BUFFER_SIZE 16
#define MAP_FRAME_RETRY_COUNT 5
typedef struct
{
    map_point_t points[MAP_POINT_BUFFER_SIZE];
    uint16_t direction;
    uint8_t count;
} real_map_points_t;


#endif


































