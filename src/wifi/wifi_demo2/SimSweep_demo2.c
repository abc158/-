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
//#include "charge.h"
#include <charge/charge.h>

#if defined(USE_WIFI_DEMO_2)
#include "simwifi_demo2.h"
#include "SimSweep_demo2.h"

#define OPEN_WIFI2_PRINT

#ifdef OPEN_WIFI2_PRINT
  #define WIFI2_PRINT(...)    printf(__VA_ARGS__);
#else
  #define WIFI2_PRINT(...)
#endif


static U16 uart_cmd = 0;
static tm  tmp_day;
static U16 map_index=0;
static u8 wifi_selftest_result = 0;
U8 send_map_flag = 0;
//U8 receive_map_ok_flag = 0;
static U8 type_three_ok=0;
tm current_day_p; 
static real_map_points_t map_points_data;

static history_map_point_t History_Points[HISTORY_MAP_POINT_SIZE];// ��ʷ��ͼbuffer
static U8 hm_write_ptr = 0; // ��ʷ��ͼдָ��
static U8 hm_read_ptr  = 0; // ��ʷ��ͼ��ָ��

static sys_state_info sys_state_info_p;  
static schedule_time_info schedule_info;
extern U8 type_three_ok;



extern U8 get_bat_level(void);
extern U8 get_remote_state(void);


extern tm *get_system_time_info(void);
extern schedule_time_info *get_system_schedule_info(void);


void send_robot_state_to_wifi(void);
void send_base_info(void)
{
   
}
extern u8 wifi_sound_flag;
void parser_wifi_state(U8 *data_buf)
{
	//printf("Parser_wifi_state\n");
	static u8 wifi_connect_flag = 0;
	if((data_buf[0]==1)&&(data_buf[1]==0)&&(data_buf[2]==0))
    {
        if(get_reset_wifi_flag() == 1)
        {
            set_reset_wifi_flag(2);
        }
        set_wifi_state(WIFI_CONNECTING); // ����״̬
    }
	
    else if((data_buf[0]==0)&&(data_buf[1]==1)&&(data_buf[2]==0))
    {
        if(get_reset_wifi_flag() == 2)
        {
            set_reset_wifi_flag(0);
        }
        set_wifi_state(WIFI_CONNECT_TO_ROUTER); // ������·����
    }	
	else if((data_buf[0]==0)&&(data_buf[1]==1)&&(data_buf[2]==1))
	{
		set_wifi_state(WIFI_CONNECT_OK);   // �ɹ����ӵ� ��̨������
		if(wifi_connect_flag==0&&wifi_sound_flag){
			wifi_sound_flag = 0;
			songplayer_play_id(SONG_ID_WIFI_SUCCESS, 0);
			wifi_connect_flag = 1;
		}
	}
    else
    {
    	set_wifi_state(WIFI_WAIT_CONNECT);
		if(wifi_connect_flag == 1&&wifi_sound_flag){
			wifi_connect_flag = 0;
			songplayer_play_id(SONG_ID_WIFI_FAIL, 0);
		}
    }
    
}



#if 0
void wifi_module_parser(U8 *data_buf)
{

    if((data_buf[0] == 2) && (data_buf[1] == 2))
    {
       // printf("wifi 1\r\n");
        wificonnectok = 0;
       // set_wifi_state(WIFI_CONNECTING);
    }
    
}
#endif

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
    send_time_cmd(); //д�������RTC,����д�����ز�����ʱ�䣬��dispд��
	/*����Ҫ������Ÿ���ʱ�䣬����������ʱ��������disp�����*/
}
u8 uart_stop_clean=0;
void parser_control_cmd(U8 *data_buf)
{
    sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);

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
		case 0x21://�������
			if(0x00 == data_buf[1])
			{
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
			/*
				���򰴼����ѣ���Ϊֻ�з��򰴼�
				��Ϊ���򰴼��������ظ����͵ģ�
				�ʺ�������������
			*/
			if(get_ui_state() == UI_ENTER_SLEEPING){
				uart_cmd = UART_START;
			}
			break;
		case 0x22://��ɨģʽ����
			if(0x00 == data_buf[1])//��ǽ
			{
				uart_cmd = UART_WALL_FOLLOW;
			}
			if(0x01 == data_buf[1])//�ص�
			{
				uart_cmd = UART_SPOT;
			}
			if(0x02 == data_buf[1])//�Զ���ɨ
			{
				if( (sys_state_info_p.robot_state == ROBOT_STATE_WAITING)
				  ||(sys_state_info_p.robot_state == ROBOT_STATE_PAUSE)
				  ||(sys_state_info_p.robot_state == ROBOT_STATE_CHARGING)
				  ||(sys_state_info_p.robot_state == ROBOT_STATE_SLEEP))
				{
					uart_cmd = UART_START;
				}
				if((sys_state_info_p.robot_state == ROBOT_STATE_WALLFOLLOW)|| \
					(sys_state_info_p.robot_state == ROBOT_STATE_DOCK)|| \
					(sys_state_info_p.robot_state == ROBOT_STATE_SPOTTING))
				{
					uart_cmd = UART_MODE1;
				}
			}
			if(0x03 == data_buf[1])
			{
				uart_cmd = UART_DOCK;
			}
			break;
		case 0x24:
			if(0x00 == data_buf[1]){
				uart_cmd = UART_NORMAL;//�������ģʽ
			}else{
				uart_cmd = UART_ENHANCE;//ǿ��ģʽ
			}
		break;
		case 0x25:
			if(0x01 == data_buf[1]){
				set_water_step(1);
			}else if(0x02 == data_buf[1]){
				set_water_step(2);
			}else if(0x03 == data_buf[1]){
				set_water_step(3);
			}
		break;
		case 0x26:
            if(0x00 == data_buf[1])
			{
				if( (sys_state_info_p.robot_state != ROBOT_STATE_WAITING)
				  &&(sys_state_info_p.robot_state != ROBOT_STATE_PAUSE)
				  &&(sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)
				  &&(sys_state_info_p.robot_state != ROBOT_STATE_SLEEP))
				  {
                    uart_stop_clean=1;
					uart_cmd = UART_STOP;
					}
			}
			if(0x01 == data_buf[1])
			{
				if( (sys_state_info_p.robot_state == ROBOT_STATE_WAITING)
				  ||(sys_state_info_p.robot_state == ROBOT_STATE_PAUSE)
				  ||(sys_state_info_p.robot_state == ROBOT_STATE_CHARGING)
				  ||(sys_state_info_p.robot_state == ROBOT_STATE_SLEEP))
				  {
					uart_cmd = UART_START;
				}
			}
			break;
			
		case 0x27:
           // send_robot_state_to_wifi();
			break;
		case 0x30:
            set_schedule(&data_buf[1]);
			break;
		case 0x31:
            set_time(&data_buf[1]);
			break;	
			
		default:
			break;
	}
}
extern u8 get_vac_state(void);
extern U16 wetmop_detect(void);
void send_robot_state_to_wifi(void)
{
    U8 i,j;
    UI_STATE_E s;

    U8 msg[27]={0}; 
    U8 tmp = get_remote_state();
    UI_ERROR_NUMBERS_E error_upload = get_ui_error_num(); 
    sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
	s = get_ui_state();
	 
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
#if 0	
	if(s != UI_ENTER_SLEEPING)
		msg[0] = 0x1; 
#endif
	msg[0] = 0xff;  // �ݲ�ʹ��
    if(s == UI_ENTER_SLEEPING)
    {
		msg[3] = 0x5; 
    }
	else if(error_upload > 0)
	{
		msg[3] = 0x9; 
    }	
	else
	{	
		switch(sys_state_info_p.robot_state)
		{
			case ROBOT_STATE_WALLFOLLOW:
				msg[3] = 0x0; //�ӱ�״̬
				break;

			case ROBOT_STATE_SPOTTING:
				msg[3] = 0x1;
				break;
			
			case ROBOT_STATE_CLEANING_ROOM:
				msg[3] = 0x2; 
				break;
			
			case ROBOT_STATE_DOCK:
				msg[3] = 0x3; 
				break;
			
			case ROBOT_STATE_WAITING:
				msg[3] = 0x4; 
				break;
				
			case ROBOT_STATE_PAUSE:
				msg[3] = 0x4; 
				break;
			case ROBOT_STATE_CHARGING:
			{
				if(sys_state_info_p.charging_state == CHARGING_COMPLETE) 
					msg[3] = 0x8;		 // ������
					
				else if(get_charging_mode())
					msg[3] = 0x7;        // ֱ��
					
				else if(!get_charging_mode())
					msg[3] = 0x6;		 // ����	
			}
				break;

			case ROBOT_STATE_REMOTE_DRIVE:
					msg[3] = 0x0a;
				break;
				
			default:
				break;
		}
	}
	msg[4] = get_water_step();
    if(wetmop_detect()){//��ˮ��
		msg[5] = 0x02;//ˮ��ģʽ�¹رշ��
	}else{
	    if(get_vac_state()==0){//��ͨģʽ
	    	msg[5] = 0x00;
	    }
		else if(get_vac_state()==1){//ǿ��ģʽ
			msg[5] = 0x01;
		}
	}
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
    
    msg[23] = schedule_info.t[0].hour;
    msg[24] = schedule_info.t[0].min;
    msg[25] = error_upload;  // sys_state_info_p.pause_id;
    msg[26] = get_bat_level();
    send_pack(CMD_GET_ROBOT_STATE, msg, 27); 
    
}

/* �����ϴ���ʧ�ĵ�ͼ���� */ 
void Resend_map_data(U8 *data_buf)
{
	u8 Point_start = 0;
	u8 Point_end   = 0;
	U8 i,j,k;
	U8 msg[85]={0}; 
	U8 point_length = 0;
	
	Point_start = data_buf[0];    
	Point_start = (((Point_start<<8)&0xff00)|data_buf[1]);    	
	Point_end   = data_buf[2];    
	Point_end	=(((Point_end<<8)&0xff00)|data_buf[3]);
	if((Point_end-Point_start) <0 )
	{
		return;
	}
	if((Point_end-Point_start+1) > 10 )
	{
		point_length = 10;
	}
	else
	{
		point_length = Point_end-Point_start+1;
	}

	msg[0] = 0x2;  //type   
	msg[1] = 0x40; // �ܵ���2byte  
	msg[2] = 0x00; 
	msg[3] = 0x00; 
    msg[4] = point_length;
    j = 0;
	for(k = 0; k < point_length; k++)
	{
	    for(i=0;i<HISTORY_MAP_POINT_SIZE;i++)
		{
			hm_read_ptr++;
			if(hm_read_ptr > HISTORY_MAP_POINT_SIZE)	hm_read_ptr = 0;
			
			if(History_Points[hm_read_ptr].his_index == Point_start)
				break;
		}

//		for(m=0;i<tmp_length;i++)
	    {			
			msg[5+j] = (History_Points[hm_read_ptr].his_index >> 8); //����2byte
	        msg[6+j] = History_Points[hm_read_ptr].his_index&0x00ff;
			
	        msg[7+j] = 0x0;
	        msg[8+j] = History_Points[hm_read_ptr].x;
				
	        msg[9+j]  = 0x0;
	        msg[10+j] = History_Points[hm_read_ptr].y;
			
	        msg[11+j] = History_Points[hm_read_ptr].type;
	        msg[12+j] = 0; 
			
	        j += 8;
	    }
		Point_start ++;
	}
	
	send_pack(CMD_MAP, msg, (j+5)); 
		  
    //printf("map:x:%d,y:%d,type:%d\r\n",save_x,save_y,save_type);

}

void send_map_data_to_wifi(void)
{
	U8 i,j;
	U8 msg[85]={0}; 
	U16 tmp_length;

	msg[0] = 0x1;  //type
	msg[1] = 0x40; // �ܵ���2byte  
	msg[2] = 0x00; //
	msg[3] = 0x00; //
  //msg[4] = 0x01; //��ǰ���ϴ�λ�õ���2byte
  if(((map_points_data.count > 0)&&(type_three_ok == 1))||(map_points_data.count>= 5))
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
    	hm_write_ptr++;
		if( hm_write_ptr >= HISTORY_MAP_POINT_SIZE )
		{	
			hm_write_ptr = 0;
		}
		
        map_index = map_index + 1;
        msg[5+j]  = (map_index >> 8); //����2byte
        msg[6+j]  = map_index&0x00ff;
		History_Points[hm_write_ptr].his_index = map_index;
		
        //AM_DEBUG_TEST("index:%d\r\n",map_index);
        msg[7+j]  = 0x0;
        msg[8+j]  = map_points_data.points[i].x;
		History_Points[hm_write_ptr].x = map_points_data.points[i].x;
			
        msg[9+j]  = 0x0;
        msg[10+j] = map_points_data.points[i].y;
		History_Points[hm_write_ptr].y = map_points_data.points[i].y;
		
        msg[11+j] = map_points_data.points[i].type;
		History_Points[hm_write_ptr].type = map_points_data.points[i].type;
        msg[12+j] = 0; 
        j += 8;
    }
	
	//printf("index=%d",map_index);
      map_points_data.count = 0;
      //send_map_flag = 1;
	  
      send_pack(CMD_MAP, msg, (j+5));
	  
	 // printf("#Send_map\n");
   //   printf("map:x:%d,y:%d,type:%d\r\n",save_x,save_y,save_type);

  }
}

/****************************************************************
*Function   :  ui_put_map_point_info
*Author     :  lyh    
*Date       :  2017.6.30
*Description:  ����ʵʱ��ͼ·������
*CallBy     :  �ɵײ���ã���ʵʱ�ĵ�ͼ·�������ϴ���Ӧ�ò�
*Input      :  ����
*              x: X ����
               y: Y ����
               type:    ��ǰ������� ��Χ��1-3 
                        1:�߹������ϰ���
                        2:���ϰ��ĵ�
                        3:��ǰ��
               direction:�����Ƕ�
*             
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
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
    if (real_map_points->count >= MAP_POINT_BUFFER_SIZE) 
	{
        for (i=0; i<MAP_POINT_BUFFER_SIZE-1; i++)
		{
            real_map_points->points[i].x = real_map_points->points[i+1].x;
            real_map_points->points[i].y = real_map_points->points[i+1].y;
            real_map_points->points[i].type = real_map_points->points[i+1].type;
        }
        real_map_points->count = MAP_POINT_BUFFER_SIZE;
    }
	else 
    {
        real_map_points->count++;
    }
    real_map_points->points[real_map_points->count-1].x = x;
    real_map_points->points[real_map_points->count-1].y = y;
    real_map_points->points[real_map_points->count-1].type = type;
    // printf("x =%d y= %d type = %d\r\n",x,y,type);
	
    if (type == MAP_POINT_TYPE_CURRENT)
	{
      real_map_points->direction = direction;
    }
}

void send_config_network_cmd(void)
{
	U8 msg[2]={0x01,0x01}; 
	send_pack(CMD_CONFIG_NETWORK, msg, 2);
	set_wifi_state(WIFI_CONNECTING);
}

void enter_wifi_selftest_state(void)
{
	send_pack(CMD_ENTER_TEST, NULL, 0); // msg   
}

void check_wifi_state(void)
{
	U8 msg[1]={0};
	send_pack(CMD_WIFI_STATE, msg, 1);  // NULL   
}

void parser_wifi_selftest_result(u8 *data_buf)
{
	if(data_buf[0] == 1)
    {
    	wifi_selftest_result = TRUE;
    }
	else
	{
		wifi_selftest_result = FALSE;
	}
}

u8 get_wifi_selftest_result(void)
{
	return wifi_selftest_result;
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
	u8 msg[1]={0x05}; 
	map_index = 0;
	type_three_ok = 0;//�������ӡ
	map_points_data.count = 0;//��buffer
	send_pack(CMD_MAP, msg, 1);//aa 02 03 05 08���ͼָ��
}

#endif

