#include <ui-config.h>
#include "ui-commands.h"
#include "am_date_base.h"
#include "am_config_macro.h"
#include "time.h"
#include "lib.h"
#include "local_key_check.h"
#include "syscall_api.h"
#include "remote.h"
#include "act.h"
#include "display.h"
#include "am_key.h"
#include "sensor/sensor.h"
#include "ui-manager/exception.h"
#include "motor/robot_suction.h"
#include "ui-song-player.h"
#include "display.h"

#include "simwifi_demo2.h"
#include "SimSweep_demo2.h"

#if defined(USE_WIFI_DEMO_2)
static U16 uart_cmd = 0;
static tm tmp_day;
static U16 map_index=0;

U8 send_map_flag = 0;
U8 receive_map_ok_flag = 0;
static U8 type_three_ok=0;
tm current_day_p; 
static real_map_points_t map_points_data;

static sys_state_info sys_state_info_p;  
static schedule_time_info schedule_info;
extern U8 wificonnectok;
extern U8 send_config_cmd;
extern U8 recive_connect_cmd_ok;
extern U8 type_three_ok;



extern U8 get_bat_level(void);
extern U8 get_remote_state(void);


extern tm *get_system_time_info(void);
extern schedule_time_info *get_system_schedule_info(void);


void send_robot_state_to_wifi(void);
void send_base_info(void)
{
   
}

void parser_wifi_state(U8 *data_buf)
{

    if(data_buf[0] == 0)
    {
       // printf("wifi 1\r\n");
        if(get_reset_wifi_flag() == 1)
        {
            set_reset_wifi_flag(2);
        }
        wificonnectok = 0;
       // set_wifi_state(WIFI_CONNECTING);
    }
    else if((data_buf[0] == 0)&&(data_buf[1] == 1)&&(data_buf[2] == 1))
    {
       // printf("wifi 3\r\n");
        if(get_reset_wifi_flag() == 2)
        {
            set_reset_wifi_flag(0);
        }
        set_wifi_state(WIFI_CONNECT_OK);
        wificonnectok = 1;
    }
    else
    {
        wificonnectok = 0;
    }
    
}

void set_schedule(U8 *pAppointBuf)  
{
    uint8_t i,j,tmp;
    schedule_time_info *schedule_info_p;
    schedule_info_p = get_schedule_info();

    tmp = (pAppointBuf[0]&0x40)>>6;
    schedule_info_p->SCH_ENABLE.BYTE = (pAppointBuf[0]<<1)|tmp;
 
    schedule_info_p->t[0].hour = pAppointBuf[13];
    schedule_info_p->t[0].min  = pAppointBuf[14];
    
    j = 1;
    for(i=1;i<7;i++)
    {
        schedule_info_p->t[i].hour = pAppointBuf[j];
        schedule_info_p->t[i].min  = pAppointBuf[j+1];
        j += 2;
    }

    send_schedule_cmd();
  
}

void set_time(U8 *data_buf)
{
    sys_info_get(SYS_TIME_INFO,(long )&current_day_p);

    /*
    tmp_day.w_year  = current_day_p.w_year;
    tmp_day.w_month = current_day_p.w_month;
    tmp_day.w_date  = current_day_p.w_date;
    */
    tmp_day.w_year  = 0;
    tmp_day.w_month = 0;
    tmp_day.w_date  = 0;
    data_buf[3] = data_buf[3] + 1;
    if(data_buf[3] == 7)
      tmp_day.week = 0;
    else
      tmp_day.week = data_buf[3];
   
    tmp_day.hour    = data_buf[0];
    tmp_day.min     = data_buf[1];
    tmp_day.sec     = data_buf[2];
    set_current_time(&tmp_day); 
    send_time_cmd(); //写入主板的RTC,这里写入主控不更新时间，在disp写入
	/*主控要隔几秒才更新时间，这里做个临时变量，在disp里调用*/
}

void parser_control_cmd(U8 *data_buf)
{
    switch(data_buf[0])
	{
		case 0x10:
			if(0x00 == data_buf[1])
			{
				uart_cmd = UART_POWER_DOWN;
			}
			if(0x01 == data_buf[1])
			{
				uart_cmd = UART_POWER_ON;
			}
			break;
		case 0x20:
			break;
		case 0x21:
			if(0x00 == data_buf[1])
			{
               // printf("forward\r\n");
				uart_cmd = UART_FORWARD;
			}
			if(0x01 == data_buf[1])
			{
				uart_cmd = UART_LEFT;
			}
			if(0x02 == data_buf[1])
			{
				uart_cmd = UART_RIGHT;
			}
			if(0x03 == data_buf[1])
			{
				uart_cmd = UART_BACK;
			}
			break;
		case 0x22:
			if(0x00 == data_buf[1])
			{
				uart_cmd = UART_WALL_FOLLOW;
			}
			if(0x01 == data_buf[1])
			{
				uart_cmd = UART_SPOT;
			}
			if(0x02 == data_buf[1])
			{
				uart_cmd = UART_START;
			}
			if(0x03 == data_buf[1])
			{
				uart_cmd = UART_DOCK;
			}
			break;
		case 0x23:
			break;
		case 0x24:
			break;
		case 0x25:
			break;
		case 0x26:
            if(0x00 == data_buf[1])
			{
				uart_cmd = UART_STOP;
			}
			if(0x01 == data_buf[1])
			{
				uart_cmd = UART_START;
			}
			break;
		case 0x27:
           // send_robot_state_to_wifi();
			break;
		case 0x28:
			break;
		case 0x40:
			break;
		case 0x42:
			break;
		case 0x30:
            set_schedule(&data_buf[1]);
			break;
		case 0x31:
            set_time(&data_buf[1]);
			break;	
	}
}

void send_robot_state_to_wifi(void)
{
    U8 i,j;
    UI_STATE_E s;
    

    U8 msg[28]={0};
    U8 tmp = get_remote_state();
   
   
    sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
     s = get_ui_state();
    if(s != UI_ENTER_SLEEPING)
    {
        msg[0]=1; 
    }

    if((sys_state_info_p.robot_state == ROBOT_STATE_CLEANING_ROOM)
        ||(sys_state_info_p.robot_state == ROBOT_STATE_PAUSE)
        ||(sys_state_info_p.robot_state == ROBOT_STATE_REMOTE_DRIVE))
    {
        msg[2]=tmp;
    }
    else
    {
        msg[2]=REMOTE_IDLE;
    }

  
    if(sys_state_info_p.robot_state == ROBOT_STATE_WALLFOLLOW)
    {
       msg[3] = 0; //延边状态
    }
    else if(sys_state_info_p.robot_state == ROBOT_STATE_CLEANING_ROOM)
    {
        msg[3] = 0x2; 
    }
    else if(sys_state_info_p.robot_state == ROBOT_STATE_SPOTTING)
    {
        msg[3] = 0x1;
    }
    else if(sys_state_info_p.robot_state == ROBOT_STATE_DOCK)
    {
        msg[3] = 0x3;
    }
    else
    {
        msg[3] = 0x4;
    }

    //msg[10] = ((schedule_info_p->SCH_ENABLE.BYTE)&0x01)<<6;
    sys_info_get(SYS_SCHEDULE_INFO, (long )&schedule_info);
    tmp = ((schedule_info.SCH_ENABLE.BYTE)&0x01)<<6;
    msg[10] =((schedule_info.SCH_ENABLE.BYTE)>>1)|tmp;
    j = 0;
    for(i=1;i<7;i++)
    {
       msg[j+11]=schedule_info.t[i].hour;
       msg[j+12]=schedule_info.t[i].min;
       j += 2;
    }
    
    msg[23]=schedule_info.t[0].hour;
    msg[24]=schedule_info.t[0].min;

    msg[25] = sys_state_info_p.pause_id;

    msg[26] = get_bat_level();

    msg[27] = songplayer_is_playing();
    
    send_pack(CMD_GET_ROBOT_STATE, msg, 28); 
    
}

void parser_map_info(U8 *data_buf)
{
    if((data_buf[0] == 0x1)||(data_buf[0] == 0x2))
    {
        //printf("receive map ok\r\n");
        send_map_flag = 0;
        receive_map_ok_flag = 1;
    }

}

void send_map_data_to_wifi(void)
{

  U8 i,j;

  U8 msg[85]={0};
  U16 tmp_length;
 
  
  msg[0] = 0x1;  //type
  msg[1] = 0x40; // 总点数2byte  
  msg[2] = 0x00; //
  msg[3] = 0x00; //
  //msg[4] = 0x01; //当前包上传位置点数2byte
  if(((map_points_data.count > 0)&&(type_three_ok == 1))
        ||(map_points_data.count>= 5))
  {
    type_three_ok = 0;

    if(map_points_data.count > 10)
    {
        tmp_length = 10;
    }
    else
    {
        tmp_length = map_points_data.count;
    }
    msg[4] = tmp_length;
    j = 0;
    for(i=0;i<tmp_length;i++)
    {
        map_index = map_index + 1;
        msg[5+j] = (map_index >> 8); //索引2byte
        msg[6+j] = map_index&0x00ff;
        //AM_DEBUG_TEST("index:%d\r\n",map_index);
        msg[7+j] = 0x0;
        msg[8+j] = map_points_data.points[i].x;

        msg[9+j]  = 0x0;
        msg[10+j] = map_points_data.points[i].y;
        msg[11+j] = map_points_data.points[i].type;
        msg[12+j]=0; 
        //printf("map_index=%d\r\n",map_index);
        /*
        if(map_points_data.points[i].type == 1)
        {
            printf("$@%d,%d!\r\n",map_points_data.points[i].x,map_points_data.points[i].y);
        }
        if(map_points_data.points[i].type == 2)
        {
            printf("$#%d,%d!\r\n",map_points_data.points[i].x,map_points_data.points[i].y);
        }
        if(map_points_data.points[i].type == 3)
        {
            printf("$&%d,%d!\r\n",map_points_data.points[i].x,map_points_data.points[i].y);
        }
        */
        j += 8;
    }
      map_points_data.count = 0;
      send_map_flag = 1;
      receive_map_ok_flag = 0;
  
      send_pack(CMD_MAP, msg, (j+5));   
     // printf("map:x:%d,y:%d,type:%d\r\n",save_x,save_y,save_type);

  }
  /*
  else if(send_map_flag = 1)
  {
      msg[5] = (map_index >> 8); //索引2byte
      msg[6] = map_index&0x00ff;
     // tmp_x = current_point->x;
     // tmp_y = current_point->y;
      msg[7] = 0x0;
      msg[8] = save_x;

      msg[9] = 0x0;
      msg[10] = save_y;
      msg[11] = save_type;
  }*/
  

}

/****************************************************************
*Function   :  ui_put_map_point_info
*Author     :  lyh    
*Date       :  2017.6.30
*Description:  保存实时地图路径数据
*CallBy     :  由底层调用，把实时的地图路径数据上传到应用层
*Input      :  参数
*              x: X 坐标
               y: Y 坐标
               type:    当前点的类型 范围是1-3 
                        1:走过的无障碍点
                        2:有障碍的点
                        3:当前点
               direction:机器角度
*             
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyh            17.6.30       v1.0         build this function
******************************************************************/
void ui_put_map_point_info(uint16_t x, uint16_t y, uint8_t type, uint16_t direction)
{
    uint8_t i;
    real_map_points_t * real_map_points;
    real_map_points = &map_points_data;
    if(type == 3)
    {
        type_three_ok = 1;
    }
    if (real_map_points->count >= MAP_POINT_BUFFER_SIZE) {
        for (i=0; i<MAP_POINT_BUFFER_SIZE-1; i++) {
            real_map_points->points[i].x = real_map_points->points[i+1].x;
            real_map_points->points[i].y = real_map_points->points[i+1].y;
            real_map_points->points[i].type = real_map_points->points[i+1].type;
        }
        real_map_points->count = MAP_POINT_BUFFER_SIZE;
    } else {
        real_map_points->count++;
    }
    real_map_points->points[real_map_points->count-1].x = x;
    real_map_points->points[real_map_points->count-1].y = y;
    real_map_points->points[real_map_points->count-1].type = type;
    if (type == MAP_POINT_TYPE_CURRENT) {
      real_map_points->direction = direction;
    }
}

void send_config_network_cmd(void)
{
   U8 msg[2]={0x01,0x01}; 
   send_pack(CMD_CONFIG_NETWORK, msg, 2);
 // send_pack(CMD_WIFI_STATE, msg, 0);
   send_config_cmd = 1;
   recive_connect_cmd_ok = 0;
}

void send_wifi_song_mute(void)
{
   U8 msg[2]={0x02,0x01}; 
   send_pack(CMD_CONFIG_NETWORK, msg, 2);
}
void send_wifi_song_unmute(void)
{
   U8 msg[2]={0x02,0x00}; 
   send_pack(CMD_CONFIG_NETWORK, msg, 2);
}
void send_wifi_test_result(void)
{
    U8 msg[1]={1};
    send_pack(CMD_ENTER_TEST, msg, 1);    
}

U16 get_uart_cmd(void)
{
  static U16 tmp;
  tmp = uart_cmd;
  uart_cmd = 0x0;
  return tmp;
}

void clear_map_index(void)
{
    map_index = 0;
}

#endif

