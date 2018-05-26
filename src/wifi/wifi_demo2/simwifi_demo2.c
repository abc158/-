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
#include "simwifi_demo2.h"
#include "SimSweep_demo2.h"
#include "charge/charge.h"
#if defined(USE_WIFI_DEMO_2)
#define WIFI_UART 1
#define WIFI_RESET_HOLD_COUNT 10 //100ms
static U16 wifi_powerup_count = 0;
U8 wifi_enable_flag;
static sys_state_info sys_state_info_p; 
static sys_current_info sys_current_info_p;   /*系统电流*/
extern U8 send_map_flag;

extern schedule_time_info  obj_schedule_info_msg_t;

/*使能wifi功能 0不使能，1使能*/
//uint8_t  enable_wifi = 1;

uint8_t  wifi_state = 0;

/*复位wifi模块标志，0不复位，1复位，2中间状态*/
uint8_t  reset_wifi_flag = 0;


uint8_t UartTxBuf[UART_TX_BUF_SIZE];//UART发送缓冲器
//uint16_t error_count = 0;
//static uint16_t  check_count = 0;

static wifi_uart_protocol_t pack;
static U8  totaLen=0;
static U8  revbuf[20];

       U8  need_send_ack = 0;

//static tm  rtc_tm;
U8 wificonnectok = 0;

U8 send_config_cmd = 0;
U8 recive_connect_cmd_ok = 0;
void Usart_SendByte(uint8_t Value);


static void check_and_powerup_wifi_module(void)
{
    if (wifi_powerup_count == 0) {
        gpio_request_one(AM_IO_WIFI_POWER,GPIO_F_DIR_OUT|GPIO_F_INIT_LOW);
        gpio_request_one(AM_IO_RESET_WIFI,GPIO_F_DIR_OUT|GPIO_F_INIT_LOW);
        gpio_set_value(AM_IO_WIFI_POWER,1);
        gpio_set_value(AM_IO_RESET_WIFI,0);
    } else if (wifi_powerup_count == WIFI_RESET_HOLD_COUNT) {
        gpio_set_value(AM_IO_WIFI_POWER,1);
        gpio_set_value(AM_IO_RESET_WIFI,1);
        
    }
    
    if (wifi_powerup_count <= WIFI_RESET_HOLD_COUNT) {
      wifi_powerup_count++;
    }
}

void wifi_handle_sleep(void)
{
    printf("wifi handle sleep###\r\n");
    //gpio_request_one(AM_IO_WIFI_POWER,GPIO_F_DIR_OUT|GPIO_F_INIT_HIGH);
    //gpio_request_one(AM_IO_RESET_WIFI,GPIO_F_DIR_OUT|GPIO_F_INIT_HIGH);
    gpio_set_value(AM_IO_WIFI_POWER,1);
    gpio_set_value(AM_IO_RESET_WIFI,1);
}

void wifi_init(void)
{
  //set_wifi_enable(1);
  //delay_ms(8);
  //uartwifi_init();
  wificonnectok = 0;
  recive_connect_cmd_ok = 0;
  check_and_powerup_wifi_module();
}


/*****************************************************
**函数体名：Usart1_SendHexString
**功能描述：串口2发送二进制数据流
**输入参数：u8 *pData,u32 Long(数组大小)
**输出参数：无 
*****************************************************/
extern void set_wifi_uart_tx_fifo_empty_irq(void);

void Usart_SendHexString(uint8_t *pHexBuf,uint32_t Long)
{
  uint32_t i;
 // printf("send:\r\n");
  for(i=0;i<Long;i++)
  {
    //senduart2(pHexBuf[i]);
   // printf("%x ",pHexBuf[i]);
    uart_txrx_q_put(WIFI_TX_INDEX,pHexBuf[i]);
  }
  //printf("\r\n");
  set_wifi_uart_tx_fifo_empty_irq();
}



static U8 pack_gen_sum(U8 *p)
{
    U8 sum = 0,i=0;

    wifi_uart_protocol_t *protocol_pack  = (wifi_uart_protocol_t *)p;
  
    if( (protocol_pack->msglen < LEN_MIN) )//长度的合法性
    {
        return 0;
    }

    sum = p[2];
   // data_buf = (U8 *)(&p[3]);
   // protocol_pack->data_buf = (U8 *)(&p[3]);
    for(i = 0;(i<protocol_pack->msglen-1);i++)
    {
       sum += protocol_pack->data_buf[i];
    }
 
    //sum = ~sum;
    //sum += 1;
  
    return sum;

}

U8 cal_pack_sum(U8 *p)
{
    U8 sum = 0,i=0;
	  U8 *data_buf;

    wifi_uart_protocol_t *protocol_pack  = (wifi_uart_protocol_t *)p;
  
    if( (protocol_pack->msglen < LEN_MIN) )//长度的合法性
    {
        return 0;
    }

    sum = p[2];
    data_buf = (U8 *)(&p[3]);
   // protocol_pack->data_buf = (U8 *)(&p[3]);
    for(i = 0;(i<protocol_pack->msglen-1);i++)
    {
  	sum += data_buf[i];
    }
 
    //sum = ~sum;
    //sum += 1;
  
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

    Usart_SendHexString(UartTxBuf,(pack.msglen+3));
    return 0;
}



static U8 pack_parser(U8 *pack)
{
	U8 cmd;
	wifi_uart_protocol_t *protocol_pack  = (wifi_uart_protocol_t *)pack;
	cmd       = protocol_pack->cmd;


    switch(cmd)
    { 
  	    //查询基础信息
  	    case CMD_CHEACK_BASE_INFO:
        {
            //send_base_info();
        }
        break;
   	    //控制指令下发
        case CMD_CONTROL:
        {
           // printf("control cmd\r\n");
            parser_control_cmd(&pack[3]);
            need_send_ack = 1;
        }
     	break;
   	    //地图指令
        case CMD_MAP:
        {
           // printf("parser map\r\n");
            parser_map_info(&pack[3]);
        }
        break;
        //WIFI网络状态查询
        case CMD_WIFI_STATE:
        {
          //  printf("parser wifi state\r\n");
            parser_wifi_state(&pack[3]);
        }
        break;
        //云平台状态查询,即查询扫地机当前的状态
        case CMD_GET_ROBOT_STATE:
        {
            send_robot_state_to_wifi();
            need_send_ack = 0;
        }
        break;  
        //配网命令
        case CMD_CONFIG_NETWORK:
        {
            //printf("wifi ACK ok\r\n");
     	    recive_connect_cmd_ok = 1;
            send_config_cmd = 0;
        }
        break; 
        //进入厂测模式
        case CMD_ENTER_TEST:
        {
     	    send_wifi_test_result();
        }
        break;        	 
        default:
     	break;
    }
    return 0;
}


static U8  bat_state = 3;
static U8  last_bat_state = 0;

U16 convert_to_adc( s16 voltage) 
{
    U32 mv = voltage<<12;
    mv = mv/18975;//0.18 ---> 18032; 0.174 --> 18975       
    return (U16)mv;
}

void check_bat_level(U16  voltage)
{
    last_bat_state = bat_state;
    if((LOW_POWER<(voltage))&&((voltage) <=MID_POWER))
	{
        bat_state = 2;
    }
	else if((MID_POWER<(voltage))&&((voltage)<=FULL_POWER))
	{
        bat_state = 1;
	}
    if((voltage)>FULL_POWER)//trick表示充电停止 //3887
    {
        bat_state = 0;
    }

    
    if(((bat_state < last_bat_state) && (sys_state_info_p.robot_state == ROBOT_STATE_CHARGING))
        || ((bat_state > last_bat_state) && (sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)))
    {
        if(last_bat_state > 0)
        {
            bat_state = last_bat_state;
        }
    }

    if(sys_state_info_p.charging_state == CHARGING_COMPLETE)
    {
        bat_state = 2;
    }
}
void handle_wifi_bat_level(void)
{
    static U16   tmp_cnt = 0;
    static U16  voltage_average = 0;
    U16 tmp_voltage = 0;
    
    sys_info_get(SYS_CURRENT_INFO,(long)&sys_current_info_p);
    sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);

    tmp_voltage = convert_to_adc( sys_current_info_p.battery_voltage) ;

    if(voltage_average == 0)
    {
        voltage_average = tmp_voltage*2;
    }
    else
    {
        voltage_average += tmp_voltage; 
    }
    
    if(voltage_average >= 3000)
    {
	    voltage_average = (voltage_average>>1);
    }
   
    tmp_cnt += 1;
    if((tmp_cnt >= 2000)||((bat_state == 3)&&(tmp_cnt >= 50)))
    {
        check_bat_level(voltage_average);
        tmp_cnt = 0;
        
    }
}
U8 uart_server_routine(void)
{
  U16 tmp=0;
  static U8 tick_timeout = 0;
 
  static DECODE_STATE_E state = DECODE_PACK_WAIT;
  static U8 i=0;


  check_and_powerup_wifi_module();
  if(wifi_powerup_count < WIFI_RESET_HOLD_COUNT)
  {
     return 0;
  }
  //接收超时
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

    switch(state)
    {
      case DECODE_PACK_WAIT:
        if(tmp == PACK_HEAD)
        {
          //  printf("aa\r\n");
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
        //printf("bb=%x\r\n",tmp);
      	i++;
      	break;
      case DECODE_PACK_READING:
        revbuf[i] = tmp;
        //printf("cc=%x\r\n",tmp);
      	i++;
        tick_timeout = 0;
      	if(i >= totaLen)
      	{
            // printf("cc=%x\r\n",tmp);
             
      	  tmp = cal_pack_sum(revbuf);
        //  printf("sum=%x\r\n",tmp);
      	  if(tmp == revbuf[totaLen-1])
      	  {
      	    pack_parser(revbuf);
      	  }	
      	  i=0;
      		totaLen = 0;
      	  state = DECODE_PACK_WAIT;	
      	  	
      	}
      	break;
      case DECODE_PACK_OK  :
      	break;
      case DECODE_PACK_ERR :
    	  break;
    } 
  }
  handle_wifi_bat_level();
  
  return 0;
}


void map_data_process(void)
{
   
    static U8 time_cnt = 0;
    static U8 last_song_state = 0;  

   
    UI_STATE_E   s = get_ui_state();
    
  
   // if(((local_map_point_available>0)||(send_map_flag > 0))/*&&(wifi_state == WIFI_CONNECT_OK)*/) 
    {
        if(send_map_flag > 0)
        {
            
           // time_cnt++;
           // if(time_cnt >= 32)
            {
               // printf("send data\r\n");
               // send_map_data_to_wifi();
                send_map_flag = 0;
               // time_cnt = 0;
            }
        }
        //else
        {
           // printf("send data\r\n");
            time_cnt++;
            if(time_cnt >= 50)
            {
                time_cnt = 0;
                send_map_data_to_wifi();
            }
        }
    }

    if(songplayer_is_playing() != last_song_state)
    {
        if(songplayer_is_playing() == 1)
        {
            send_wifi_song_mute();
        }
        else
        {
            send_wifi_song_unmute();
        }
    }
   
    last_song_state = songplayer_is_playing();
}

U8 get_bat_level(void)
{
    return bat_state;
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
    wifi_state = value;
}

uint8_t get_wifi_state(void)
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
 //   set_wifi_enable(0);  
}

#endif

