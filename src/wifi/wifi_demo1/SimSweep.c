
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

#include "simwifi.h"
#include "simsweep.h"

#if defined(USE_WIFI_DEMO_1)

#define Big_Endian
//****************************************
uint8_t Status_WifiDevice;
//******以下为仿真扫地机内部变量，客户根据自身程序修改
uint8_t  Sim_oData0[4]={0,0,0,0};           //0 状态数据项 扫地机状态信号,32bit
uint8_t  Sim_oData1[8]={0,0,0,0,0,0,0,0};   //1 状态数据项 扫地机当前预约参数,64bit 
uint8_t  Sim_oData2[8]={0,0,0,0,0,0,0,0};   //2 状态数据项 扫地机当前预约参数,64bit
uint8_t  Sim_oData3[4]={0X55,0X66,88,0};    //3 状态数据项 扫地机产品代码,电池电量，音乐音量
uint8_t  Sim_oData4[1]={0x08};           //0X54~0X54,UV,MP3存在性，及相关下载参数
uint8_t  Sim_oData5[1]={0x01};           //0X55~0X55,地图编号
uint16_t Sim_E2ROM_OEMCode=0X0000;          //OEM_CODE
//************************RTC**********************
uint16_t year=0;  //年 
uint8_t mon=0;   //月
uint8_t day=0;   //日
uint8_t hour=0;  //时
uint8_t min=0;   //分 
uint8_t Sec=0;   //秒
uint8_t week=0;  //星期几
uint8_t LeapYearFlag=0;//润年标志 =0平,=1润
//*************************************************

//******以下为仿真扫地机地图变量，客户根据自身程序修改

uint16_t  Map_Sim_Index=0;            //扫地机当前位置索引

uint8_t   Sim_oData0x70_0x8F[32]={0};//扫地机推送位置数据，用户需按实际赋值

uint8_t   Sim_oData0x90_0xAF[3][32]={0};//扫地机获取位置数据，用户需按实际赋值

uint8_t   Sim_oData0xB0_0xCF[32]={0};//扫地机历史地图数据，用户需按实际赋值


uint8_t   Sim_oData6[8]={0,0,0,0,0,0,0,0};   //获取位置1缓存
uint8_t   Sim_oData7[8]={0,0,0,0,0,0,0,0};   //获取位置2缓存 
uint8_t   Sim_oData8[8]={0,0,0,0,0,0,0,0};   //获取位置3缓存
uint8_t   Sim_oData9[8]={0,0,0,0,0,0,0,0};   //获取位置4缓存


//************************RTC**********************

static tm tmp_day;
static U16 uart_cmd = 0;


static U8 init_time = 0;
//static U8 sos_state = 0;
//static U8 power_state = 0;
//real_map_points_t *current_point;
static real_map_points_t map_points_data;
U16 current_index;
 U8 type_three_ok=0;
extern U8 clean_index;

bitmap_info_t BitmapMsg;  
t_map_info *map_info=NULL;
uint8_t tmp_length=0;
uint8_t send_cur_data_flag = 1;
uint8_t send_old_data_flag = 0;
uint8_t send_history_map_flag = 0;
uint16_t history_index=0,history_cnt=0;
static sys_state_info sys_state_info_p;      /*系统状态*/

//U16 map_index = 0;
U8  modetype_code[3]={0x00,0x10,0x20};//,0x20,0x10,0x08,0x04}; 
U8  send_save_his_flag = 0;

U32  time_count = 0; 
U32  time_count2 = 0;
U8  save_map_index = 0;
U8  target_index = 0;
U16 wifi_uart_cmd0[]=
{
    UART_FORWARD,
    UART_LEFT,
    UART_RIGHT,
    UART_CMD_NULL,
    UART_VOL_SUB,
    UART_VOL_ADD,
    UART_CMD_NULL,
    UART_CMD_NULL
};

U16 wifi_uart_cmd1[]=
{
    UART_DOCK,
    UART_START,
    UART_ENHANCE,
    UART_CMD_NULL,
    UART_MODE1,
    UART_MODE2,
    UART_CMD_NULL,
    UART_CMD_NULL
    
};

U16 wifi_uart_cmd2[]=
{
    UART_START,
    UART_STOP,
    UART_SOS_ON,
    UART_SOS_OFF,
    UART_MPLAY_ON,
    UART_MPLAY_OFF,
    UART_UV_ON,
    UART_UV_OFF
    
};

U16 wifi_uart_cmd3[]=
{
    UART_POWER_ON,
    UART_POWER_DOWN,
    UART_SCHEDULE,
    UART_MODE2,
    UART_PLAY_MUSIC,
    UART_STOP_MUSIC,
    UART_GET_PRODUCT_ID,
    UART_GET_SCHDULE_INFO
};

void Action_0X00(uint8_t AppKeySig0X00)  //1Byte 控制命令下发
{
    uint8_t i;
    
    for(i=0;i<8;i++)
    {
        if(AppKeySig0X00&(1<<i))
        {
           uart_cmd = wifi_uart_cmd0[i];
        }
    }
}
void Action_0X01(uint8_t AppKeySig0X01)  //1Byte 控制命令下发
{
    uint8_t i;

    for(i=0;i<8;i++)
    {
        if(AppKeySig0X01&(1<<i))
        {
            uart_cmd = wifi_uart_cmd1[i];

	                
        }
    }
}
void Action_0X02(uint8_t AppKeySig0X02)  //1Byte 控制命令下发
{
    uint8_t i;

    for(i=0;i<8;i++)
    {
        if(AppKeySig0X02&(1<<i))
        {
			uart_cmd = wifi_uart_cmd2[i];
	    /*
            if(uart_cmd == UART_SOS_ON)
            {
                sos_state = 1;
            }
            else
            {
                sos_state = 0;
            }
            */
        }
    }
}
void Action_0X03(uint8_t AppKeySig0X03)  //1Byte 控制命令下发
{
    uint8_t i;

    for(i=0;i<8;i++)
    {
        if(AppKeySig0X03&(1<<i))
        {
            uart_cmd = wifi_uart_cmd3[i];
        }
    }
}
extern uint8_t MapFunFlag;
void Action_0X04(uint8_t AppKeySig0X04)  //1Byte 用户修改代码
{
 if(AppKeySig0X04&0X01) ;
   if(AppKeySig0X04&0X02) ;
     if(AppKeySig0X04&0X04) ;              
       if(AppKeySig0X04&0X08);       
          if(AppKeySig0X04&0X10) ;
          if(AppKeySig0X04&0X20)
          {MapFunFlag=ONCE; }
      
          if(AppKeySig0X04&0X40)
          {
             Sim_oData4[0]|=0X10;
             if(send_cur_data_flag == 0)
             {
                 send_cur_data_flag = 1;
                 time_count = timer_ms();
			     printf("open curdata!!!!!\r\n");
             }
          }//扫地机推送功能打开
          if(AppKeySig0X04&0X80)
          {
             Sim_oData4[0]&=~0X10;
             printf("close curdata>>>>>\r\n");
             send_cur_data_flag = 0;
          }//扫地机推送功能关闭
          
}

extern U8 MapDatCount;

void Action_0X05(uint8_t AppKeySig0X05)  //1Byte 用户修改代码
{

 if(AppKeySig0X05&0X01) ;
   if(AppKeySig0X05&0X02) ;
     if(AppKeySig0X05&0X04) ;              
       if(AppKeySig0X05&0X08);       
          if(AppKeySig0X05&0X10) ;
          if(AppKeySig0X05&0X20){/*MapFunFlag=ONCE;*/ }
          if((AppKeySig0X05&0X40))//获取历史数据预处理
          {
              
              Sim_oData4[0]|=0X04;
          
              if(send_history_map_flag == 0)
              {
                 printf("############SSopen history!!!!!\r\n");
                 MapDatCount=MapDatDelay;
                 history_index = 0xffff;
                 memset(Sim_oData0xB0_0xCF,0x00,32);
                 BitmapMsg.err_map_data_flag = 0;
                 BitmapMsg.update_his_map_flag = 0;
                 send_save_his_flag=0;
             
                 time_count2 = timer_ms();
                 
              }
              send_history_map_flag = 1;
          }//扫地机位置历史推送功能打开
          if(AppKeySig0X05&0X80)
          {
              Sim_oData4[0]&=~0X04;
              if(send_history_map_flag == 1)
			 {
                  send_history_map_flag = 0;
                  BitmapMsg.err_map_data_flag = 0;
                 // err_map_data_count = 0;
                  send_save_his_flag=0;
                  memset(Sim_oData0xB0_0xCF,0x00,32);
                  printf("***********SSclose history----\r\n");
			 }
             time_count2 = timer_ms();
          }//扫地机位置历史推送功能关闭
}

 //*************1V3_Begin 
void Action_0X1A(uint8_t AppKeySig0X1A)  //音量APP 设定
{
    uint8_t tmp;
    tmp = AppKeySig0X1A;
    tmp = tmp*3;
//    set_mplayer_volume(tmp);
}
void Action_0X08_0X17(uint8_t *pAppointBuf,uint8_t len)  //16Byte 预约处理 用户修改代码
{
    uint8_t i,j,tmp;
    schedule_time_info *schedule_info_p;
    schedule_info_p = get_schedule_info();

    tmp = (pAppointBuf[0]&0x40)>>6;
    schedule_info_p->SCH_ENABLE.BYTE = (pAppointBuf[0]<<1)|tmp;
 
    schedule_info_p->t[0].hour = pAppointBuf[14];
    schedule_info_p->t[0].min  = pAppointBuf[15];
    
    j = 2;
    for(i=1;i<7;i++)
    {
        schedule_info_p->t[i].hour = pAppointBuf[j];
        schedule_info_p->t[i].min  = pAppointBuf[j+1];
        j += 2;
    }

    send_schedule_cmd();
  
}
void Action_0X20_0X21(uint8_t *pAppointBuf,uint8_t len)  //2Byte 实时点补点处理
{
    uint8_t  i;
  
   
    for(i=0;i<3;i++)
    {
        if((Sim_oData0x90_0xAF[i][0] == pAppointBuf[0]) &&
            (Sim_oData0x90_0xAF[i][1] == pAppointBuf[1]))
        {
            send_old_data_flag = 1;
            target_index = i;
            break;
        }
     
    }
    
    if(i==3)
    {
        send_old_data_flag = 0;
    }

}

void Action_0X20_0X25(uint8_t *pAppointBuf,uint8_t len)  //4Byte 历史数据补漏处理
{
    U16 TMP_X,TMP_Y;

    if(send_save_his_flag == 0)
    {
        
        BitmapMsg.param.frame_h = pAppointBuf[0];
        BitmapMsg.param.frame_l = pAppointBuf[1];
        TMP_X = BitmapMsg.param.frame_h;
        TMP_Y = BitmapMsg.param.frame_l;
        
        BitmapMsg.param.start_x = pAppointBuf[2];
        BitmapMsg.param.start_y = pAppointBuf[3];
        BitmapMsg.param.end_x   = pAppointBuf[4];
        BitmapMsg.param.end_y   = pAppointBuf[5];
        if((pAppointBuf[2]!=Sim_oData0xB0_0xCF[2])
            &&(pAppointBuf[3]!=Sim_oData0xB0_0xCF[3])
            &&(pAppointBuf[4]!=Sim_oData0xB0_0xCF[4])
            &&(pAppointBuf[5]!=Sim_oData0xB0_0xCF[5])
            )
        {
            if(pAppointBuf[5]>=pAppointBuf[3])
            {
               printf("#### no ok index!!\r\n");
                memset(Sim_oData0xB0_0xCF,0x00,32);
                send_save_his_flag = 1;
            }
            else
            {
                TMP_Y = pAppointBuf[5];
               // printf("error y2=%u\r\n",TMP_Y);
            }
        }
        else
        {
            //TMP_X = Sim_oData0xB0_0xCF[0];
            //TMP_Y = Sim_oData0xB0_0xCF[1];
            printf("&&&&&ok index,index0=%d,index1=%d\r\n",TMP_X,TMP_Y);
            memset(Sim_oData0xB0_0xCF,0x00,32);
        }
        
        time_count2 = timer_ms();
    }
}

void Action_0X18_0X19(uint16_t OEMCode)  //2Byte 用户修改代码
{
  Sim_E2ROM_OEMCode=OEMCode;//仿存入E2ROM,
}

void Action_0X1B_0X1E(uint8_t *pAppointBuf)  //4Byte 获取索引处理 用户修改代码
{
  
   //仿获取索引参数,位置获取期间，判断历史数据起点 和终点
   // Map_Sim_LowIndex=pAppointBuf[0]<<8|pAppointBuf[1];
  
   // Map_Sim_HighIndex=pAppointBuf[2]<<8|pAppointBuf[3];
    
	 // tmp_mappoint.start_x = pAppointBuf[0];
	 // tmp_mappoint.start_y = pAppointBuf[1];
	 // request_map_points();
}

void Action_0XF1(uint8_t WifiCardStatusReg)  //1Byte 可用作Wifi指示灯,判断wifi是否连接上
{

    if(((WifiCardStatusReg&0X03)==0X00)&&((WifiCardStatusReg&0X0c)==0X08)&&((WifiCardStatusReg&0X20)==0x20))
    {
        Status_WifiDevice=5;
		
        if((get_reset_wifi_flag()) == 2)
        {
            set_reset_wifi_flag(0);
            songplayer_play_id(SONG_ID_WIFI_CONNET_OK, 0);
        }

    }
    else if((get_reset_wifi_flag() == 1))
    {
        set_reset_wifi_flag(2);    //one step
			  //letter_load(1,2);
    }
}
void Action_0XF8_0XFF(uint8_t *pRTCBuf,uint8_t len,uint8_t APPinstalledEvent)  //16Byte RTC处理
{
  //计算本地RTC（变换时区后）
  int8_t TimeZoneHour;//时区小时差，含正负值
  uint8_t TimeZoneMin;//时区分钟差,绝对值，根据小时差选正负号

  if((APPinstalledEvent)||(init_time == 0))//设备发现事件时，校对RTC时间
  {
    TimeZoneHour=*(int8_t *)pRTCBuf;//时区差
    TimeZoneMin=pRTCBuf[1];         
    year=pRTCBuf[2];            //年
    year+=2000;                 //加千位数   
    mon=pRTCBuf[3];             //月
    mon>>=4;
    day=pRTCBuf[4];             //日
    hour=(pRTCBuf[5]&0X7F);     //时
    min=pRTCBuf[6];             //分 
    Sec=pRTCBuf[7];             //秒
    week=(pRTCBuf[3]&0X0F);  //星期几=0,星期日,=1,星期一,...=6，星期六

                                  //判断平润
    if((year%4==0&&year%100!=0)||year%400==0)
    {
        LeapYearFlag=1; //润年
     }else
     {
        LeapYearFlag=0;//平年   
     }    
                              //校时区差
    if(TimeZoneHour>0)//标准时间+TimeZoneHour+TimeZoneMin
    {
      min+=TimeZoneMin;if(min>=60) {min-=60;hour++;}
      hour+=TimeZoneHour;//加小时
      if(hour>=24) 
        { 
          hour-=24; //进一日          
          DayDayADD1();
        }
    }      
    else if(TimeZoneHour<0) //标准时间+TimeZoneHour-TimeZoneMin
    {
      hour+=24;//预借位1天，24小时
      min+=60;//预借位1小时,60分钟
      min-=TimeZoneMin;//计算当前分钟数
      if(min>=60)
        min-=60;//退预借60分钟
      else 
        hour--;//扣除1小时     
      hour+=TimeZoneHour;//加负数小时
      
      if(hour>=24)
        hour-=24;//退预借24小时
      else 
        DayDaySub1();//减一天     
    }
    //*****************用户可增加写入RTC代码*********
    if(year != 2000)
    {
        tmp_day.w_year  = 0;
        tmp_day.w_month = 0;
        tmp_day.w_date  = 0;
        tmp_day.week    = week;
        tmp_day.hour    = hour;
        tmp_day.min     = min;
        tmp_day.sec     = Sec;
        set_current_time(&tmp_day); 
        send_time_cmd(); //写入主板的RTC,这里写入主控不更新时间，在disp写入
        printf("week:%d hour:%d min:%d\r\n",week,hour,min);
        init_time = 1;
    }
   // init_time = 1;
  }
}

void DayDayADD1(void)//加一天
{
    uint8_t MaxDayM=0;//当月最大天数
    if(week==6)
    {
        week=0;
    }
    else 
    {
        week++;//星期几，校正
    }
   // MaxDayM=GetMaxDayM(mon);//取当月最大天数        
    if(++day>MaxDayM)
    { 
        day=1;
        mon++;
    }//进1月
    if(mon>12) 
    {
        year++;
        mon=1;
    }//进一年,12月31日 ==>1月1日
    
}
void DayDaySub1()//减一天
{     
    uint8_t PreMon;//上个月
    PreMon=mon-1;
    if(PreMon==0) PreMon=12;//上个月，是去年12月    
    if(week==0) week=6;else week--;//校正星期几  
    if(--day==0) { day=GetMaxDayM(PreMon);if(--mon==0) {mon=12;--year;}};//退1月或退一年 (1月1日的前一天)==>> 12月31日 
     
}
uint8_t GetMaxDayM(uint8_t mon_8t)//取某月的最大天数
{
    uint8_t MaxDayM=0;//当月最大天数
    if((mon_8t==1)||(mon_8t==3)||(mon_8t==5)||(mon_8t==7)||(mon_8t==8)||(mon_8t==10)||(mon_8t==12))
      MaxDayM=31;
    else if(mon_8t==2)
    { MaxDayM=28;if(LeapYearFlag) MaxDayM=29;}   
    else MaxDayM=30;
    return MaxDayM;
}

//********************************************

//********************************************

void  Get_oData0_0X50_0X53(uint8_t *p_oData0Buf,uint8_t BufByteLen) //上传 系统状态到APP
{
//  uint8_t i;
  //SYSTEM_STATE_E s = get_system_state();
  //U8 mode_type = get_mode_type();
 // sys_state_info *sys_state_info_p; 

  U8 mode_type = 0;
  UI_STATE_E   s = get_ui_state();
  
   
  sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);

  p_oData0Buf[0] = sys_state_info_p.pause_id;
  
    
  p_oData0Buf[1] = 0x00;
  
  
  if(s == UI_ENTER_SLEEPING)
  {
     p_oData0Buf[1]|=0X01;  
     wifi_send_sleep_flag = 1;
    // printf("wifi enter sleep!!!!!\r\n");
  }
  if((sys_state_info_p.robot_state == ROBOT_STATE_WAITING))
  {
      p_oData0Buf[1]|=0X02;
  }
  if((sys_state_info_p.robot_state == ROBOT_STATE_CLEANING_ROOM)||
    (sys_state_info_p.robot_state == ROBOT_STATE_SPOTTING))
  {
      p_oData0Buf[1]|=0X08;
  }
  if(sys_state_info_p.pause_id > 0)
  {
      p_oData0Buf[1]|=0X10;
  }
  //if(s == SYS_MPLAY)
  p_oData0Buf[4] = 0x08; 

  if(send_cur_data_flag == 1)
  {
     p_oData0Buf[4] |= 0x10;
  }

  if(send_history_map_flag == 1)
  {
     p_oData0Buf[4] |= 0x04;
  }
  p_oData0Buf[2] = 0x00;
  /*
  if(sos_state == 1)
  {
      p_oData0Buf[2]|=0X01;
  }*/
  if(mode_type > 0)
  {
     // p_oData0Buf[2]|= modetype_code[mode_type]; 
  }
  else if(sys_state_info_p.robot_state == ROBOT_STATE_CHARGING)
  {
      if(mode_type == 0)
      {
          p_oData0Buf[2]|=0X02;
      }
      /*
      else
      {
          p_oData0Buf[2]|= modetype_code[mode_type];
      }*/
  }


  p_oData0Buf[3] = 0x00;
  if(sys_state_info_p.robot_state == ROBOT_STATE_CHARGING)
  {
      p_oData0Buf[3] |= 0X08;  
      //p_oData0Buf[3] |= 0X20;  
  }
  else if(sys_state_info_p.robot_state == ROBOT_STATE_DOCK)
  {
      p_oData0Buf[3] |= 0X01;
  }
  else
  {
     p_oData0Buf[3] |= 0x04;
  }


}



void  Get_oData1_0X58_0X5F(uint8_t *p_oData1Buf,uint8_t BufByteLen)// 上传 预约数据到APP
{
  uint8_t tmp;
  schedule_time_info *schedule_info_p;
  schedule_info_p = get_schedule_info();

  tmp = ((schedule_info_p->SCH_ENABLE.BYTE)&0x01)<<6;
  p_oData1Buf[0] = ((schedule_info_p->SCH_ENABLE.BYTE)>>1)|tmp;

  p_oData1Buf[2] = schedule_info_p->t[1].hour;
  p_oData1Buf[3] = schedule_info_p->t[1].min;

  p_oData1Buf[4] = schedule_info_p->t[2].hour;
  p_oData1Buf[5] = schedule_info_p->t[2].min;

  p_oData1Buf[6] = schedule_info_p->t[3].hour;
  p_oData1Buf[7] = schedule_info_p->t[3].min;
  

}  
void  Get_oData2_0X60_0X67(uint8_t *p_oData2Buf,uint8_t BufByteLen)// 上传 预约数据到APP
{
//  uint8_t i;
  schedule_time_info *schedule_info_p;
  schedule_info_p = get_schedule_info();
  
  p_oData2Buf[0] = schedule_info_p->t[4].hour;
  p_oData2Buf[1] = schedule_info_p->t[4].min;

  p_oData2Buf[2] = schedule_info_p->t[5].hour;
  p_oData2Buf[3] = schedule_info_p->t[5].min;

  p_oData2Buf[4] = schedule_info_p->t[6].hour;
  p_oData2Buf[5] = schedule_info_p->t[6].min;

  p_oData2Buf[6] = schedule_info_p->t[0].hour;
  p_oData2Buf[7] = schedule_info_p->t[0].min;

}

void  Get_oData3_0X68_0X6B(uint8_t *p_oData3Buf,uint8_t BufByteLen)//get oData3
{
//  uint8_t i;
  *(U16 *)(p_oData3Buf)=0xc001;
 // p_oData3Buf[2] = get_now_electricity(); 
  p_oData3Buf[2] = get_bat_level(); 
//  p_oData3Buf[3] = get_mplayer_volume(); 
  p_oData3Buf[3] = 0;
  //for(i=0;i<BufByteLen;i++) //输给APP 娄据项Sim_oData1
    //p_oData3Buf[i]=Sim_oData3[i];  
}
void  Get_oData4_0X54_0X54(uint8_t *p_oData4Buf,uint8_t BufByteLen)//get oData4,
{
    p_oData4Buf[0] = 0x08; 
}
void  Get_oData5_0X55_0X55(uint8_t *p_oData5Buf,uint8_t BufByteLen)//get oData5,
{
  uint8_t i;
  for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData5
    p_oData5Buf[i]=clean_index;
}

void  Get_oData6_0X70_0X77(uint8_t *p_oData6Buf,uint8_t BufByteLen)//get oData5,上传地图实时数据到APP
{
//	int tmp;
  uint8_t i,j;


  if(time_count == 0)
  {
     time_count = timer_ms();
  }
  if(timer_elapsed(time_count) > 1500)
  {
     // printf("$$$$$$$$$send data++++++\r\n");
      if(timer_elapsed(time_count) > 1600)
      {
          time_count = timer_ms();
      }
    //  tmp_length = map_rx_get_datalength();
  
      if(((map_points_data.count > 0)&&(send_history_map_flag == 0)&&(type_three_ok == 1))
        ||(map_points_data.count >= 5))
      {
          //test_cnt = test_cnt + 1;
          type_three_ok = 0;
          time_count = 0;
          Map_Sim_Index = Map_Sim_Index + 1;
          if(map_points_data.count >= 7)
          {
              tmp_length = 7;
          }
          else
          {
             tmp_length = map_points_data.count;
          }
          if(++save_map_index >= 3)
          {
              save_map_index = 0;
          }
          memcpy(Sim_oData0x90_0xAF[save_map_index],Sim_oData0x70_0x8F,32);
          memset(Sim_oData0x70_0x8F,0x00,32);
          Sim_oData0x70_0x8F[0] = Map_Sim_Index>>8;
          Sim_oData0x70_0x8F[1] = Map_Sim_Index&0x00ff;
          j=2;
          //printf("send: ");
          for(i=0;i<tmp_length;i++)
          {
              //current_point = map_rx_q_get();
               
              Sim_oData0x70_0x8F[j] = map_points_data.points[i].type;
              Sim_oData0x70_0x8F[j+1] = map_points_data.points[i].x;
              Sim_oData0x70_0x8F[j+2] = map_points_data.points[i].y;

              /*
              tmp_type = Sim_oData0x70_0x8F[j];
              tmp_x = Sim_oData0x70_0x8F[j+1];
              tmp_y = Sim_oData0x70_0x8F[j+2];
        
              printf("%d ",tmp_x);
              printf("%d ",tmp_y);
              printf("%d ",tmp_type);
              */
             j = j + 3;
          }
          map_points_data.count = 0;
          
          // printf("\r\n");
          //Action_0X05(0x00);
          time_count = timer_ms();
      }
      
     // printf("time_count=%u\r\n",time_count); 
  }
 // printf("send data++++++++++++++++++\r\n");
  {
      for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData5
       p_oData6Buf[i]=Sim_oData0x70_0x8F[i];
  }
}

void  Get_oData7_0X78_0X7F(uint8_t *p_oData7Buf,uint8_t BufByteLen)//get oData7,上传地图实时数据到APP2
{
  uint8_t i;
  {
     for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x70_0x8F
     p_oData7Buf[i]=Sim_oData0x70_0x8F[i+8];
  }
}


void  Get_oData8_0X80_0X87(uint8_t *p_oData8Buf,uint8_t BufByteLen)//get oData6，上传地图实时数据到APP3
{
    uint8_t i;

    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x70_0x8F
        p_oData8Buf[i]=Sim_oData0x70_0x8F[i+8+8];
    }
      
}  
void  Get_oData9_0X88_0X8F(uint8_t *p_oData9Buf,uint8_t BufByteLen)//get oData7，上传地图实时数据到APP4
{
    uint8_t i;

    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData9Buf[i]=Sim_oData0x70_0x8F[i+8+8+8];
    }
} 
void  Get_oData10_0X90_0X97(uint8_t *p_oData10Buf,uint8_t BufByteLen)//get oData8，上传丢失的地图实时数据到APP
{
    uint8_t i;
    
    if(send_old_data_flag > 0)
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData10Buf[i]=Sim_oData0x90_0xAF[target_index][i];
    }
    else
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData10Buf[i]=0x00;
    }
    /*
    for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData10Buf[i]=Sim_oData0x90_0xAF[i];*/
    
} 
void  Get_oData11_0X98_0X9F(uint8_t *p_oData11Buf,uint8_t BufByteLen)//get oData9，上传丢失的地图实时数据到APP2
{
    uint8_t i;
    
    if(send_old_data_flag > 0)
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData11Buf[i]=Sim_oData0x90_0xAF[target_index][i+8];
    }
    else
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData11Buf[i]=0x00;
    }
    //for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
      //  p_oData11Buf[i]=Sim_oData0x90_0xAF[i+8];
} 

void  Get_oData12_0XA0_0XA7(uint8_t *p_oData12Buf,uint8_t BufByteLen)//get oData12，上传丢失的地图实时数据到APP3
{
    uint8_t i;
    
    if(send_old_data_flag > 0)
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData12Buf[i]=Sim_oData0x90_0xAF[target_index][i+8+8];
    }
    else
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData12Buf[i]=0x00;
    }
    //for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
      //  p_oData12Buf[i]=Sim_oData0x90_0xAF[i+8+8];
    
}

void  Get_oData13_0XA8_0XAF(uint8_t *p_oData13Buf,uint8_t BufByteLen)//get oData13，上传丢失的地图实时数据到APP4
{
    uint8_t i;
    
    if(send_old_data_flag > 0)
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData13Buf[i]=Sim_oData0x90_0xAF[target_index][i+8+8+8];
        send_old_data_flag = 0;
    }
    else
    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
        p_oData13Buf[i]=0x00;
    }
    //for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0x90_0xAF
      //  p_oData13Buf[i]=Sim_oData0x90_0xAF[i+8+8+8];
    
}

void  Get_oData14_0XB0_0XB7(uint8_t *p_oData14Buf,uint8_t BufByteLen)//get oData14,上传地图历史数据到APP
{
    uint8_t i;  

    if((send_history_map_flag > 0)&&(BitmapMsg.update_his_map_flag > 0))
    {
       // bitmap_info = get_bitmap_info();
        Sim_oData0xB0_0xCF[0] = BitmapMsg.param.frame_h;
        Sim_oData0xB0_0xCF[1] = BitmapMsg.param.frame_l;
        Sim_oData0xB0_0xCF[2] = BitmapMsg.param.start_x;
        Sim_oData0xB0_0xCF[3] = BitmapMsg.param.start_y;
        Sim_oData0xB0_0xCF[4] = BitmapMsg.param.end_x;
        Sim_oData0xB0_0xCF[5] = BitmapMsg.param.end_y;
        memcpy(&Sim_oData0xB0_0xCF[6],BitmapMsg.bitdata,26);
        //printf("send Get_oData14_0XB0_0XB7\r\n");
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0xB0_0xCF
        p_oData14Buf[i]=Sim_oData0xB0_0xCF[i];
    }
    else
    {
        //if(send_history_map_flag == 0)
        if((send_history_map_flag > 0)&&(BitmapMsg.err_map_data_flag > 0))
        {
            
            {
                printf("^^^^^^^^^^^^** send 0x80 $$\r\n");
                BitmapMsg.err_map_data_flag = 0;
                //Sim_oData0xB0_0xCF[0] = 0x80;
               // memset(Sim_oData0xB0_0xCF,0x00,32);
               // send_history_map_flag = 0;
                history_index = 0;
                BitmapMsg.update_his_map_flag = 0;
                send_save_his_flag=0;
                time_count2  = timer_ms();
            }
        }
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0xB0_0xCF
        p_oData14Buf[i]=Sim_oData0xB0_0xCF[i];
    }
}

void  Get_oData15_0XB8_0XBF(uint8_t *p_oData15Buf,uint8_t BufByteLen)//get oData15,上传地图历史数据到APP2
{
    uint8_t i;

    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0xB0_0xCF
        p_oData15Buf[i]=Sim_oData0xB0_0xCF[i+8];
    }
}

void  Get_oData16_0XC0_0XC7(uint8_t *p_oData16Buf,uint8_t BufByteLen)//get oData16,上传地图历史数据到APP3
{
    uint8_t i;


    {
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0xB0_0xCF
        p_oData16Buf[i]=Sim_oData0xB0_0xCF[i+8+8];
    }

}

void  Get_oData17_0XC8_0XCF(uint8_t *p_oData17Buf,uint8_t BufByteLen)//get oData17,上传地图历史数据到APP4
{
    uint8_t i;
    U16 tmp_len = 0;
    U16 x1,y1,x2,y2;
    if((send_history_map_flag > 0)&&(BitmapMsg.update_his_map_flag > 0))
    {
        // printf("send Get_oData17_0XC8_0XCF\r\n");
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0xB0_0xCF
        p_oData17Buf[i]=Sim_oData0xB0_0xCF[i+8+8+8];
        x1=Sim_oData0xB0_0xCF[0];
        y1=Sim_oData0xB0_0xCF[1];
        //printf("index0=%d index1=%d\r\n",x1,y1);
        if((Sim_oData0xB0_0xCF[0]==0x00)&&(Sim_oData0xB0_0xCF[1]==0x00))
        {
            x1 = Sim_oData0xB0_0xCF[2];
            y1 = Sim_oData0xB0_0xCF[3];
            x2 = Sim_oData0xB0_0xCF[4];
            y2 = Sim_oData0xB0_0xCF[5];
            tmp_len = (abs(Sim_oData0xB0_0xCF[3]-Sim_oData0xB0_0xCF[5]))*(abs(Sim_oData0xB0_0xCF[4]-Sim_oData0xB0_0xCF[2]));
            printf("tmp_len=%d\r\n",tmp_len);
            printf("x1=%d,y1=%d,x2=%d,y2=%d\r\n",x1,y1,x2,y2);
            if(tmp_len%104 > 0)
            {
                history_cnt = tmp_len/104 + 1;
            }
            else
            {
                history_cnt = tmp_len/104;
            }
            tmp_len = history_cnt;
            
            printf("history_cnt=%d\r\n",tmp_len);
       
        }
        history_index = history_index + 1;
        BitmapMsg.update_his_map_flag = 0;
        
    }
    else
    {
        if((send_history_map_flag > 0)&&(send_save_his_flag==0))
        {
            if(timer_elapsed(time_count2) > 600)
            {
                if(history_index == 0xffff)
                {
                    history_index = 0;
                    history_cnt   = 1;
                    //tmp_map_request_info = get_history_map_info();
                    BitmapMsg.param.frame_h = 0;
                    BitmapMsg.param.frame_l = 0;
                    BitmapMsg.param.start_x = 0;
                    BitmapMsg.param.start_y = 0;
                    BitmapMsg.param.end_x   = 255;
                    BitmapMsg.param.end_y   = 255;
                    //request_history_map_info();
                }
                else
                {
                    
                    //history_index = 0xff00;
                    BitmapMsg.param.frame_h = history_index>>8;
                    BitmapMsg.param.frame_l = history_index&0x00ff;
                    BitmapMsg.param.start_x = Sim_oData0xB0_0xCF[2];
                    BitmapMsg.param.start_y = Sim_oData0xB0_0xCF[3];
                    BitmapMsg.param.end_x   = Sim_oData0xB0_0xCF[4];
                    BitmapMsg.param.end_y   = Sim_oData0xB0_0xCF[5];
                }
                x1 = BitmapMsg.param.frame_h;
                y1 = BitmapMsg.param.frame_l;
                printf("***************frame_h=%d,frame_l=%d\r\n",x1,y1);
        
                sys_info_get(SYS_BITMAP_INFO, (long )&BitmapMsg);
                //request_history_map_info();
                //uartTx();
                time_count2 = timer_ms();
            }
        }

        if((send_save_his_flag == 1)&&(timer_elapsed(time_count2) > 500))
        {
            if((Sim_oData0xB0_0xCF[2] == 0)&&(Sim_oData0xB0_0xCF[3]==0)
                &&(Sim_oData0xB0_0xCF[4]==0)&&(Sim_oData0xB0_0xCF[5]==0))
            {
                sys_info_get(SYS_BITMAP_INFO, (long )&BitmapMsg);
                send_save_his_flag = 0;
                //uartTx();
                //printf("444444request_history_map_info!!!\r\n");
            }
            else
            {
                send_save_his_flag = 0;
               // printf("222222his_flag time out!!!\r\n");
            }
            
            //send_save_his_flag = 0;
            time_count2 = timer_ms();
        }
        for(i=0;i<BufByteLen;i++) //输给APP 数据项Sim_oData0xB0_0xCF
        p_oData17Buf[i]=Sim_oData0xB0_0xCF[i+8+8+8];
    }
  
}

//*******************
uint16_t  Get_OEM_CODE(void)//取OEM CODE
{
    //用户加电初始化,需要重E2ROM中加载此值。

     return Sim_E2ROM_OEMCode;
}

#define Index_Mask_C      63
#define Index_MaskNum_C   6 

//******************************************************************************


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


U16 get_uart_cmd(void)
{
  static U16 tmp;
  tmp = uart_cmd;
  uart_cmd = 0;
  return tmp;
}

U8 *get_his_map_buffer(void)
{
    return Sim_oData0xB0_0xCF;
}

void clear_map_index(void)
{
    Map_Sim_Index = 0;
}

#endif

