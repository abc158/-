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
#include "ui_uart_handle/mqueue.h"
#include "monitor/robot_batter.h"
#include "am_uart_api.h"

#if defined(USE_WIFI_DEMO_2)

#include "simwifi_demo2.h"
#include "SimSweep_demo2.h"
#include "charge/charge.h"

//#define YUGONG_SEND_DEBUG
//#define YUGONG_REC_DEBUG


#define WIFI_UART 1
#define WIFI_RESET_HOLD_COUNT 10 //100ms
static U16 wifi_powerup_count = 0;
U8 wifi_enable_flag;
u8 Power_On_Flag = 0;
static sys_state_info sys_state_info_p; 
extern u16 battery_voltage_average(void);
extern schedule_time_info  obj_schedule_info_msg_t;

extern SYS_STATE_CHG_E sys_state_4_wifi;
extern u8 low_batt_flag;
extern SYS_STATE_CHG_E sys_state_4_wifi;

WIFI_STATE_E wifi_state = WIFI_WAIT_CONNECT;

/*��λwifiģ���־��0����λ��1��λ��2�м�״̬*/
uint8_t  reset_wifi_flag = 0;
extern BOOLEAN  state_changed;   //  TRUE : ϵͳ״̬�иı䣬�������������

uint8_t UartTxBuf[UART_TX_BUF_SIZE];//UART���ͻ�����
//static SendStateType send_state = SEND_NONE_CMD;
SendStateType send_state = SEND_NONE_CMD;

 U8  send_ack_enable_flag=0;

static wifi_uart_protocol_t pack;
static U8  totaLen=0;
static U8  revbuf[50];
u8  calc_ele_rightnow = 0;
extern u8 wake_to_rxwifidata;

void Usart_SendByte(uint8_t Value);

static void check_and_powerup_wifi_module(void)
{
    if (wifi_powerup_count == 0)
	  {     
        gpio_request_one(AM_IO_WIFI_POWER,GPIO_F_OUT_INIT_HIGH);
        gpio_request_one(AM_IO_RESET_WIFI,GPIO_F_OUT_INIT_LOW);
        gpio_set_value(AM_IO_WIFI_POWER,1);
				gpio_set_value(AM_IO_RESET_WIFI,1);
    } 
    if (wifi_powerup_count <= WIFI_RESET_HOLD_COUNT) 
		{
      wifi_powerup_count++;
    }
}

void wifi_handle_sleep(void)
{
    gpio_set_value(AM_IO_WIFI_POWER,1);
    gpio_set_value(AM_IO_RESET_WIFI,1);
}

void wifi_init(void)
{
  check_and_powerup_wifi_module();
}


/*****************************************************
**����������Usart1_SendHexString
**��������������2���Ͷ�����������
**���������u8 *pData,u32 Long(�����С)
**����������� 
*****************************************************/
extern void set_wifi_uart_tx_fifo_empty_irq(void);

void Usart_SendHexString(uint8_t *pHexBuf,uint32_t Long)
{
	uint32_t i;
	for(i=0;i<Long;i++)
	{
		lock_irq();
		uart_txrx_q_put(WIFI_TX_INDEX,pHexBuf[i]);
		unlock_irq();
	}
	
	set_wifi_uart_tx_fifo_empty_irq();
}



static U8 pack_gen_sum(U8 *p)
{
    U8 sum = 0,i=0;
    wifi_uart_protocol_t *protocol_pack  = (wifi_uart_protocol_t *)p;
  
    if( (protocol_pack->msglen < LEN_MIN) )//���ȵĺϷ���
    {
        return 0;
    }

    sum = p[2];
    for(i = 0;(i<protocol_pack->msglen-1);i++)
    {
       sum += protocol_pack->data_buf[i];
    }
  
    return sum;
}

U8 cal_pack_sum(U8 *p)
{
	U8 sum = 0,i=0;
	U8 *data_buf;
    wifi_uart_protocol_t *protocol_pack  = (wifi_uart_protocol_t *)p;
  
    if( (protocol_pack->msglen < LEN_MIN) )//���ȵĺϷ���
    {
        return 0;
    }

    sum = p[2];
    data_buf = (U8 *)(&p[3]);
    for(i = 0;(i<protocol_pack->msglen-1);i++)
    {
  	sum += data_buf[i];
    }  
    return sum;
}

U8 send_pack(U8 cmd, U8 *data_buf, U8 datalen)
{
    U8 i;
    U8 *p = (U8*)&pack;
    memset((U8 *)p, 0, sizeof(wifi_uart_protocol_t));
    pack.head        = PACK_HEAD;
    pack.msglen      = datalen + 1;
    pack.cmd         = cmd;
    pack.data_buf    = data_buf;
    pack.checksum    = pack_gen_sum((U8*)&pack);
    memset(UartTxBuf,0x00,sizeof(UartTxBuf));
    for(i = 0;i< LEN_OF_BASE_PACK;i++)
    {
        UartTxBuf[i] = p[i];
    }
  
    for(i = 0;i<datalen;i++)
    {
        UartTxBuf[(i+LEN_OF_BASE_PACK)] = pack.data_buf[i];   
    }
	
    UartTxBuf[(i+LEN_OF_BASE_PACK)] = pack.checksum ;
	if	(get_ui_state() != UI_TEST||(cmd == 0x0a))//����ģʽ��ֻ�г���ָ��Ŵ���
	{
		#ifdef YUGONG_SEND_DEBUG
	    printf("UartTxBuf\r\n");
	    for(i=0;i<(pack.msglen+3);i++){
			printf("%d ",UartTxBuf[i]);
		}
		printf("\r\n");
		#endif
	    Usart_SendHexString(UartTxBuf,(pack.msglen+3));
	}
    return 0;
}
void sendTask(void)
{    
    if(send_ack_enable_flag==1)
    {       
        send_ack_enable_flag=0;
        switch (send_state)
        {
        	case SEND_DEVICE_MAP:
				send_map_data_to_wifi();
            break;
			case SEND_DEVICE_MAP_RESEND:
				if(sys_state_4_wifi==SYS_STATE_CHANGE_OK)
            	Resend_map_data(&revbuf[3]);
            break;
        	case SEND_DEVICE_STATE:
			send_robot_state_to_wifi();
			state_changed = 0;
            break;
        	default:
            break;
        }
		send_state = SEND_NONE_CMD;
    }
}


static U8 parseCommand(U8 *pack)
{
	U8 cmd;
	wifi_uart_protocol_t *protocol_pack  = (wifi_uart_protocol_t *)pack;
	cmd       = protocol_pack->cmd;
    switch(cmd)
    { 
  	    //��ѯ������Ϣ
  	    case CMD_CHEACK_BASE_INFO:
        {
        }
        break;
		case CMD_UART_RESET:{
			//wifi_uart_reset();//UART��ʼ������ʱ���UART ����ƫ��
		}
		break;
   	    //����ָ���·�
        case CMD_CONTROL:
        {
            parser_control_cmd(&pack[3]);
        }
     	break;
/***********��ͼָ��***************/
        case CMD_MAP:
        {
					send_state = SEND_DEVICE_MAP_RESEND;
					send_ack_enable_flag = 1;
        }
        break;
/***********WIFI����״̬��ѯ***************/
        case CMD_WIFI_STATE:
        {
            parser_wifi_state(&pack[3]);
        }
        break;
/****** ��ƽ̨״̬��ѯ,����ѯɨ�ػ���ǰ��״̬ *******/
        case CMD_GET_ROBOT_STATE:
        {
            send_state = SEND_DEVICE_STATE;//״̬����
            send_ack_enable_flag = 1;
        }
        break;  
/***********��������***************/
        case CMD_CONFIG_NETWORK:
        {
        }
        break; 
/*********** ���볧��ģʽ ***************/
        case CMD_ENTER_TEST:
        {
					parser_wifi_selftest_result(&pack[3]);
        }
        break;  
        default:
	     	break;
    }
    return 0;
}



/******************************************************************

* �����������(ͨ���¶��жϳ���)��ADC:3800(17.008v),�����Ӻ�:ADC:3791
* �͵�س��ѹ:3195(14.3V)20% 	CHARGING_LOW_VOLTAGE
* �������͹ػ�:2949(13.2) 0%    CHARGING_CUTOFF_VOLTAGE
  // 3030 --> ��Ϊ: 2950-13.2v
	bat_voltage = Vadc*3.3/4096/0.18
	
	adc = bat_voltage*0.18*4096/3.3
********************************************************************/
U16 voltage_adc[] = 
{
	/*0%   20%   40%    60%    80%   100%  */ 
  //   2790, 3230, 3370, 3510, 3670 ,3730   // 3708(16.6v)
    2790, 3230, 3370, 3470, 3570 ,3670   // 3708(16.6v) 
};

U16 charge_voltage_adc[] = 
{ 
  /*0%    20%    40%   60%    80%   100%  */ 
  //2830, 3270, 3410, 3540, 3700,3753
  2830, 3270, 3400, 3500, 3600,3753
};
#define NOMAL_SPORT 40
#define WATER_SPORT  20
static U8  now_electricity = 0;
static U8  last_electricity = 0;
void calc_electricity(U16  voltage)
{
    U8  i = 0;
    static u8 last_i = 0;
    static u8 last_i_cnt = 0;
    static U8  last_state;

    last_electricity = now_electricity;
    sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);

    for(i=0;i<6;i++)
    {
    	//�����˶�״̬�µĵ�ص�ѹ�ж�
        if( sys_state_info_p.robot_state == ROBOT_STATE_CLEANING_ROOM || \
			sys_state_info_p.robot_state == ROBOT_STATE_WALLFOLLOW || \
			sys_state_info_p.robot_state == ROBOT_STATE_DOCK || \
			sys_state_info_p.robot_state == ROBOT_STATE_REMOTE_DRIVE )
		{
        	if(!wetmop_detect())//��ˮ��
            {
	            if((voltage_adc[i] - NOMAL_SPORT)> voltage)  
	            {
	                break;
	            }
        	}
			else
			{
				if((voltage_adc[i] - WATER_SPORT)> voltage)  
	            {
	                break;
	            }
        	}
    	}
        else if(sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)
        {
            if(voltage_adc[i] > voltage)  
            {
               break;
            }
        }
        else
        {
            if(charge_voltage_adc[i] > voltage)
            {
				break;
            }
        }
    }
	/***���õ�ѹ�ȼ��ڱ߽�ֵʱ����***/
	//�����ȼ��仯ֻ�е�����50�βɼ��㶨������仯
	if(last_i==0){
		last_i = i;//������ʼʱ��last_i��ֵ
	}else{
		if(last_i!=i){
			if(last_i_cnt<50){
				last_i_cnt++;//�ȼ��仯�Ǵ�
				i = last_i;
			}else{
				last_i = i;
				last_i_cnt = 0;
			}
		}else{
			last_i_cnt = 0;
		}
	}
	/*************************/
    if(i == 0)
    {
        now_electricity = 10;
    }
    else if(i == 6)
    {
        now_electricity = 100;
    }
    else
    {
        now_electricity = 20*i; // 20*(i-1);
    }		

/*****����״̬�²�ȡ����ֵ��ĵ�ص�ѹֵ��ʹ������ǰ����ĵ��������ϱ�******/
	if(sys_state_info_p.robot_state==ROBOT_STATE_SLEEP){
		now_electricity = last_electricity;
	}
	if(get_ui_state() == UI_ENTER_SLEEPING)    
	{    
		now_electricity = last_electricity;
	}
/*************************************************************************/	
	if((last_state!= ROBOT_STATE_CHARGING)&&(sys_state_info_p.robot_state==ROBOT_STATE_CHARGING))
    {
        now_electricity = last_electricity;
    }
    else if((last_state==ROBOT_STATE_CHARGING)&&(sys_state_info_p.robot_state!=ROBOT_STATE_CHARGING))
    {
        now_electricity = last_electricity;
    }
	last_state = sys_state_info_p.robot_state;
	if((sys_state_info_p.charging_state == CHARGING_COMPLETE)&&(sys_state_info_p.robot_state == ROBOT_STATE_CHARGING))
    {
        now_electricity = 100;
    }		
}
U8 get_bat_level(void)
{
	static u8 old_electricity = 0xff;
	if(old_electricity==0xff){
		old_electricity = now_electricity;
        }
	if(old_electricity != now_electricity){
		if(low_batt_flag == 1){//�͵�س��־
			old_electricity = 20;
			//return 20;
		}else if(get_electricity_drop_flag()==0){//ֻ���½�
			if(old_electricity>now_electricity){
				old_electricity = now_electricity;
			}
		}else if(get_electricity_drop_flag()==1){//ֻ������
			if(old_electricity<now_electricity){
				old_electricity = now_electricity;
			}	
		}
	}
    return old_electricity;
}
void uart_char_decode(u16 rev)
{
	static u8 rev_judge = 0;	
	switch(rev_judge)
	{
		case 0:
			if(rev == 'r')	
			{
				rev_judge++;
			}
			break;

		case 1:
			if(rev == 'e')	
			{
				rev_judge++;
			}
			else			
				rev_judge = 0;

			break;

		case 2:
			if(rev == 'v')	
			{
				rev_judge++;
			}
			else			
				rev_judge = 0;
			break;

		case 3:
			if(rev == ' ')	
			{
				rev_judge++;
			}
			else	rev_judge = 0;
			break;

		case 4:
			if(rev == 'e')	
			{
				rev_judge++;
			}
			else	rev_judge = 0;
			break;
			
		case 5:
			if(rev == 'r')	
			{
				rev_judge++;
			}
			else	rev_judge = 0;
			break;
			
		case 6:
			if(rev == 'r')	
			{
				rev_judge++;
			}
			else	rev_judge = 0;
			
			break;

		case 7:
			if(rev == 'o')	
			{
				rev_judge++;
			}
			else	rev_judge = 0;
			break;

		case 8:
			if(rev == 'r')	
			{
				rev_judge = 0;
			}
			else	rev_judge = 0;
			break;

			
		default:
		break;

	}
}
u8 deal_flag = 0;
u8 wifi_data_read(void)//���ݶ�ȡ
{
	u8 tmp=0;
	static U8 i=0;
	static U8 tick_timeout = 0;
	static DECODE_STATE_E state = DECODE_PACK_WAIT;
		//���ճ�ʱ
	if(deal_flag==1){
		i=0;
		totaLen = 0;
		state = DECODE_PACK_WAIT;
		deal_flag = 0;
	}
	tick_timeout++;
	if(tick_timeout >= 220)
	{
		uart_txrx_reset_writeptr(WIFI_RX_INDEX);
		uart_txrx_reset_readptr(WIFI_RX_INDEX);
		state = DECODE_PACK_WAIT;
		tick_timeout = 0;
	}
	while(uart_txrx_q_empty(WIFI_RX_INDEX)==0)
	{
		tmp = uart_txrx_q_get(WIFI_RX_INDEX);
		uart_char_decode(tmp);
		switch(state)	
		{
		  case DECODE_PACK_WAIT:
			if(tmp == PACK_HEAD)
			{
				i = 0;
				totaLen= 0;
				state = DECODE_PACK_HEAD;
				revbuf[0]= tmp;
				i++;
			}	  
			tick_timeout = 0;
			break;
		  case DECODE_PACK_HEAD:
				state = DECODE_PACK_READING;
				totaLen = tmp+3;
				revbuf[1] = tmp;
				tick_timeout = 0;
				i++;
			break;
		  case DECODE_PACK_READING:
			revbuf[i] = tmp;
			i++;
			tick_timeout = 0;
			if(i >= totaLen)
			{
				tmp = cal_pack_sum(revbuf);
			#ifdef YUGONG_REC_DEBUG
				printf("sum=%x\r\n",tmp);
			#endif
			//Ϊ�����޹�ģ��У���뷢�ʹ���,����ģʽ���ݲ�У��
			//(revbuf[totaLen-3]==0x0a)   
				if((tmp == revbuf[totaLen-1]) || \
					(revbuf[totaLen-3]==0x0a))
				{
					#ifdef YUGONG_REC_DEBUG
					if(revbuf[2]!=5){
						for(i=0;i<totaLen;i++){
							printf("%d ",revbuf[i]);
						}
						printf("\r\n");
					}
					#endif
					//����2��ʾ����δ�����겻�ڽ�������
					//deal_flag = 2;
					return 1;
				}else{
					deal_flag = 1;//���ݴ�����������Ҫ������Ŷ
				}
			}
			break;
		  case DECODE_PACK_OK  :
			break;
		  case DECODE_PACK_ERR :
			  break;
		} 
	  }
	return 0;

}
void state_data_process(void)
{
	static u16  tmp_cnt = 0;
	static u16  voltage_average = 0;
	static u8   first_check_voltage_flag = 0;
	static u16  Reported_robot_state_count = 0;
	static u8 Reported_robot_sleep_state_cnt = 0;
	/***************************��������********************************/
	tmp_cnt += 1;
    if((tmp_cnt >= 300)||(!first_check_voltage_flag)||calc_ele_rightnow)
    {
		voltage_average = battery_voltage_average();
		calc_electricity(voltage_average);
		first_check_voltage_flag = 1;
		calc_ele_rightnow = 0;
		tmp_cnt = 0;
	}
	/***********************************************************************/
	/***************************״̬�ϱ�*******************************/
	if(state_changed)//״̬�л�
	{
		send_state = SEND_DEVICE_STATE;
		send_ack_enable_flag = 1;
		Reported_robot_state_count 		= 0;
		Reported_robot_sleep_state_cnt 	= 0;
	}else{
		if(get_ui_state()!=UI_ENTER_SLEEPING)
		{
			if(++Reported_robot_state_count > 1000)  
			{
				send_state = SEND_DEVICE_STATE;
				send_ack_enable_flag = 1;
				Reported_robot_state_count 		= 0;
				Reported_robot_sleep_state_cnt 	= 0;
			}
		}
		else
		{
			if((++Reported_robot_state_count > 100)&&(Reported_robot_sleep_state_cnt < 10))  
			{	
				send_state = SEND_DEVICE_STATE;
				send_ack_enable_flag = 1;
				Reported_robot_sleep_state_cnt ++;
				Reported_robot_state_count = 0;
			}
		}
	}

}
U8 uart_server_routine(void)
{
	check_and_powerup_wifi_module();
	if(wifi_powerup_count < WIFI_RESET_HOLD_COUNT)
	{
		return 0;
	}
	//״̬�����ϱ�����
	state_data_process();
	//��ͼ��ʱ�ϱ�����
	#ifdef USE_WIFI_DEMO_2
	if(sys_state_4_wifi==SYS_STATE_CHANGE_OK)
	//״̬�л������в������ͼ�����ϱ�
	  map_data_process(0);
	else{
	  map_data_process(1);
	}
	#endif

	if(wifi_data_read())//��ȡ��һ���ʺϵ�����
		{
		parseCommand(revbuf);//��������
		deal_flag = 1;
		}
	return 0;
}

void map_data_send_timer(void)
{
	send_ack_enable_flag = 1;
	send_state = SEND_DEVICE_MAP;
}

void map_data_process(u8 flag)
{
	static u8 last_state;
  	sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
 	if (last_state != sys_state_info_p.robot_state)
	{
		if(	 ((sys_state_info_p.robot_state==ROBOT_STATE_CLEANING_ROOM) && \
			 (last_state!=ROBOT_STATE_PAUSE))  || \
			 (IS_CLEANNING_MODE(sys_state_info_p.robot_state))  )
	   	{
        	timer_task_register(map_data_send_timer, 500, TRUE);//���붨ʱ��
	   	}
		if(!IS_CLEANNING_MODE(sys_state_info_p.robot_state)){
			timed_tasks_unregister(map_data_send_timer);//���붨ʱ
		}
	}
	last_state = sys_state_info_p.robot_state;
}

void set_wifi_enable(uint8_t value)
{
	if(value)
		wifi_enable_flag=1;
	else
		wifi_enable_flag=0;	
}

uint8_t get_wifi_enable_state(void)
{
    return wifi_enable_flag;
}

void set_wifi_state(uint8_t value)
{
    wifi_state = (WIFI_STATE_E)value;
}

WIFI_STATE_E get_wifi_state(void)
{
    return wifi_state;
}
void set_reset_wifi_flag(uint8_t value)
{
    reset_wifi_flag = value;
}

uint8_t get_reset_wifi_flag(void)
{
    return reset_wifi_flag;
}
void exit_wifi(void)
{   
}

#endif



