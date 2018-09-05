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
#include "simwifi.h"
#include "simsweep.h"
#include "charge/charge.h"
#if defined(USE_WIFI_DEMO_1)
#define WIFI_UART 1


//extern data U8 Fg_System20msSig; //移植时，需引入
//extern bit wifi_enable_flag;
extern u16 sys_uart_trig_tx_evnet(u16 uart_index);
extern schedule_time_info  obj_schedule_info_msg_t;
static U32 wifi_delay_ts = 0;
uint16_t Value_HeaterPower=0;
/*使能wifi功能 0不使能，1使能*/
//uint8_t  enable_wifi = 1;
/*复位wifi模块标志，0不复位，1复位，2中间状态*/
uint8_t  reset_wifi_flag = 0;
uint8_t  wifi_enable_flag = 0;
uint8_t  wifi_send_sleep_flag =0;
uint8_t UartRxBuf[UART_RX_BUF_SIZE];//UART接收缓冲器
uint8_t P_RxBufWrite=0;//UART接收缓冲器,写指针
uint8_t fg_RXFrame=ZERO_FRAME;//UART接收帧,状态
uint8_t UartTxBuf[UART_TX_BUF_SIZE];//UART发送缓冲器
uint16_t rxLenBuff = 0;
//uint16_t error_count = 0;

static sys_state_info sys_state_info_p; 
static sys_current_info sys_current_info_p;   /*系统电流*/
void Usart1_SendByte(uint8_t Value);
/****************************************************
**函数体名：Delay_nms
**功能描述：毫秒级延时
**输入参数：u32 m
**输出参数：无   
*****************************************************/

#if 0
void Delay_N_mS(uint32_t m)
{
  uint16_t i,j;

  for(i=0;i<m;i++)
  {
    for(j=0;j<1200;j++);
  }
}
#endif

void ResetUartRxBuf(void)
{
#ifdef gzsdebug 
   uint8_t i;
    for(i=0;i<UART_RX_BUF_SIZE;i++)
   {
     UartRxBuf[i]=0;
   }
#endif   
   fg_RXFrame=ZERO_FRAME;//reset fg_RXFrame信号
   P_RxBufWrite=0;//reset P_RxBufWrite
 }
#ifdef gzsdebug 
void ResetUartTxBuf(void)
{
   uint8_t i;
    for(i=0;i<UART_TX_BUF_SIZE;i++)
   {
     UartTxBuf[i]=0;
   }
 }
#endif 

//********************
#ifdef gzsdebug
uint8_t DataTest=0;
#endif 
//****************




#define iData0Adr 0X00    //0 数据项 APP按键信号
//#define iData0BitLen 32    
//***********1V3_Begin
#define iData0BitLen    48    
//***********1V3_End


//***************1V3_Begin
#define iData8Adr       0X1A    //8 数据项 0x1A 音量
#define iData8BitLen    8   
//***************1V3_End

//**********************以上,可插入新的数具项
  
//***************1V3_Begin
#define iData8Index (iData7Index+iData7BitLen/8)
//***************1V3_End

//**********************以上,可插入新的数具项  



/***********Eden code start************/
typedef struct
{
    uint8_t datalen;
    uint8_t * p_buff;
} mapData_t;

mapData_t m_mapData;

uint16_t m_curMapId = 0;

typedef struct
{
    uint8_t batLevl;
    uint8_t alarmInfo;
    uint8_t fanStat;
    uint8_t sweepDrit;
    uint8_t cleanTime;
    uint8_t workMode;
    uint8_t workStat;
    uint8_t sysStat;
}sendStatDat_t;
sendStatDat_t m_statDat;

void Usart1_SendHexString(uint8_t *pHexBuf,uint32_t Long);
void sendMapDat( mapData_t * p_mapDataArr )  
{
  uint16_t len = 0;
  uint16_t checksum = 0;
  
  UartTxBuf[0] = 0XB5;
  UartTxBuf[1] = 0X62;
  len = p_mapDataArr->datalen + 1 + 2;//ID + checksum +Count + id
  UartTxBuf[2] = (uint8_t)((len & 0xff00) >>8);           //LEN_H
  UartTxBuf[3] = (uint8_t)(len & 0xff);                 //LEN_L
  UartTxBuf[4] = TxMapDatFuncCode;             //ID
  UartTxBuf[5] = p_mapDataArr->datalen/6;          //count
  UartTxBuf[6] =(uint8_t)((m_curMapId & 0xff00)>>8);     //mapId_H
  UartTxBuf[7] = (uint8_t)(m_curMapId&0xff);            //mapId_L 
  memcpy(&UartTxBuf[8],p_mapDataArr->p_buff, p_mapDataArr->datalen);
  checksum = calCheckSum(UartTxBuf,len);
  UartTxBuf[8+p_mapDataArr->datalen] = (uint8_t)((checksum &0xff00)>>8);
  UartTxBuf[9+p_mapDataArr->datalen] = (uint8_t)(checksum&0xff); 
  Usart1_SendHexString(UartTxBuf,len+4);
}


//发送状态数据
void sendStatDat( uint8_t * p_statData)
{
    uint16_t len = 0;
    uint16_t checksum = 0;
    
    UartTxBuf[0] = 0XB5;
    UartTxBuf[1] = 0X62;
    UartTxBuf[2] = 0x00;
    UartTxBuf[3] = 11;
    UartTxBuf[4] = TxStatDatFuncCode;                //命令码ID

    memcpy(&UartTxBuf[5],p_statData,11-2);
    
    checksum = calCheckSum(UartTxBuf,11);

    UartTxBuf[13] = (uint8_t)((checksum & 0xff00)>>8);
    UartTxBuf[14] = (uint8_t)(checksum & 0xff); 
    Usart1_SendHexString(UartTxBuf,len+4);
}

//发送厂测命令

void sendFactorCmd( void)
{
   //B5 62 00 04 03 03 00 00
    uint16_t len = 0;
    uint16_t checksum = 0;
    UartTxBuf[0] = 0XB5;
    UartTxBuf[1] = 0X62;
    UartTxBuf[2] = 0X00;
    UartTxBuf[3] = 0X04;
    UartTxBuf[4] = 0X03;
    UartTxBuf[5] = 0X03;
    UartTxBuf[6] = 0X00;
    UartTxBuf[7] = 0X00;
    Usart1_SendHexString(UartTxBuf,8);
}

//发送取消绑定的命令
void sendDelBondCmd( void)
{
    uint16_t len = 0;
    uint16_t checksum = 0;
    //B5 62 00 04 03 02 00 00
    UartTxBuf[0] = 0XB5;
    UartTxBuf[1] = 0X62;
    UartTxBuf[2] = 0X00;
    UartTxBuf[3] = 0X04;
    UartTxBuf[4] = 0X03;
    UartTxBuf[5] = 0X02;
    UartTxBuf[6] = 0X00;
    UartTxBuf[7] = 0X00;
    Usart1_SendHexString(UartTxBuf,8);
}

//发送配网命令
void sendSetNetCmd( void)
{
    //B5 62 00 04 03 01 00 00
    uint16_t len = 0;
    uint16_t checksum = 0;
    UartTxBuf[0] = 0XB5;
    UartTxBuf[1] = 0X62;
    UartTxBuf[2] = 0X00;
    UartTxBuf[3] = 0X04;
    UartTxBuf[4] = 0X03;
    UartTxBuf[5] = 0X01;
    UartTxBuf[6] = 0X00;
    UartTxBuf[7] = 0X00;
    Usart1_SendHexString(UartTxBuf,8);
}


/***********Eden code end************/


uint8_t DataModeFlag=1;

void UART_Config_PinResetWifiCard(void)
{
  volatile uint32_t time = 2000000;
  init_wifi_module(); 
  while(time--);
  powerup_wifi_module(); 
  //InsertExtCmd(ParaSaveToWifiE2ROM);//初始化，组件号，首个命令
  wifi_delay_ts = timer_ms();
}

/*****************************************************
**函数体名：Usart2_SendByte
**功能描述：串口2发送1字节
**输入参数：Value 
**输出参数：无 
*****************************************************/
void Usart1_SendByte(uint8_t Value)
{
  // senduart2(Value);
}

/*****************************************************
**函数体名：Usart1_SendHexString
**功能描述：串口2发送二进制数据流
**输入参数：u8 *pData,u32 Long(数组大小)
**输出参数：无 
*****************************************************/
extern void set_wifi_uart_tx_fifo_empty_irq(void);
void Usart1_SendHexString(uint8_t *pHexBuf,uint32_t Long)
{
  uint32_t i;
 // printf("send: ");
  for(i=0;i<Long;i++)
  {
   // printf("%x ",pHexBuf[i]);
    uart_txrx_q_put(WIFI_TX_INDEX,pHexBuf[i]);
  }
 // printf("\r\n");
  set_wifi_uart_tx_fifo_empty_irq();
  //sys_uart_trig_tx_evnet(WIFI_UART);
}

//void SendCmdRegPack(uint8_t cmd)
//{
//   StructureWifiCmdRegPack(cmd);
//   Usart1_SendHexString(UartTxBuf,UartTxBuf[2]);
//}

//void SendAPPDisplaySigPack(void )
//{
//   StructureAPPDisplaySigPack();
//   Usart1_SendHexString(UartTxBuf,UartTxBuf[2]);
//} 


/*****************************************************
**函数体名：IRQ_Usart2RxData_Process
**功能描述：中断串口2接收数据处理
**输入参数：无
**输出参数：无 
*****************************************************/
uint8_t FrameLen=0; 
uint16_t FrameAddSum=0;
void IRQ_Usart1RxData_Process(uint8_t value)
{
  uint8_t Dat;
  uint16_t check_sum=0;
  //UART1_GetITStatus( UART1_IT_OR);
  Dat=value; //readme Uart  
  
  if((fg_RXFrame==FRAME_ERROR)||(fg_RXFrame==FRAME_OK)) //stop RecFrame 直到main处理了这个帧信号 
         return;   
 // printf("[Eden]Buf=%2x, cnt = %d\r\n",UartRxBuf[P_RxBufWrite],P_RxBufWrite);
  switch(P_RxBufWrite)
  {
  case 0:
      if(Dat==0XB5) //frame head syn
      {
        UartRxBuf[P_RxBufWrite]=Dat;
        FrameAddSum=0XAA;
        P_RxBufWrite++;  
      }
      break;
  case 1:  
      if(Dat==0X62)//frame head syn
      {
        UartRxBuf[P_RxBufWrite]=Dat;
        FrameAddSum+=Dat;       
        P_RxBufWrite++;
       // check_count = 0;
      }
      else
      {
        P_RxBufWrite=0;//reset P_RxBufWrite
      }
      break;
  case 2:                   //frame len hs
     rxLenBuff = 0xff00 & ((uint16_t)Dat<<8);
     UartRxBuf[P_RxBufWrite]=Dat;
     P_RxBufWrite ++;
     break;
  case 3:
      rxLenBuff = rxLenBuff | (Dat & 0xff);
      //printf("[Eden]Len = %d\n\r",rxLenBuff);
      if(rxLenBuff<3)                       //FrameLen<3
      {
           rxLenBuff = 0x0000;
           P_RxBufWrite=0;           //reset P_RxBufWrite       
      }
      else
      {  
          FrameLen=(uint8_t)rxLenBuff;
          UartRxBuf[P_RxBufWrite]=Dat;
          P_RxBufWrite++;
      }
      break;    
  default:    
      UartRxBuf[P_RxBufWrite]=Dat;   //uart 接收中 
      P_RxBufWrite++;
      if((FrameLen+4)== P_RxBufWrite )//
      {   
          check_sum = (UartRxBuf[FrameLen+4-2]<<8)|(UartRxBuf[FrameLen+4-1]);

          for(int i=0;i<FrameLen+4;i++)
            printf("[Eden]Buf=%2x, cnt = %d\r\n",UartRxBuf[i],i);
          printf("checksum=0x%4x\r\n",check_sum);
          
          if(check_sum ==calCheckSum(UartRxBuf,FrameLen-2) ) 
          {  
              fg_RXFrame=FRAME_OK;//UART接收帧,发出Frame_OK信号
#if 1
              printf("[Eden]FRAME_OK\n\r");
#endif
          }
          else
            fg_RXFrame=FRAME_ERROR;//UART接收帧,累加和错       
      }
      else if (P_RxBufWrite>FrameLen+4)
      {  
          fg_RXFrame=FRAME_ERROR;   //UART接收帧,发出FrameError信号  
          P_RxBufWrite=0;           //reset P_RxBufWrite
      }        
      break; 
  }      
  if(P_RxBufWrite>(UART_RX_BUF_SIZE-1))//超出接收缓冲器大小,需调整UARTRXBUFSIZE大小
  {
    P_RxBufWrite=0;                     //reset P_RxBufWrite
  }  
  
}


/*****************************************************
**函数体名：ReadData_From_UartRxBuf
**功能描述：从接收缓冲区读取数据,且处理
**输入参数：CmdAckFun_8,ack功能码
**输出参数：反回ACK状态
*****************************************************/
uint8_t ReadData_From_UartRxBuf(uint8_t CmdAckFun_8)
{
  //printf("[Eden]->ReadData_From_UartRxBuf,cmd=0x%x\r\n",CmdAckFun_8);
  
  if(fg_RXFrame==ZERO_FRAME) //wait 
        return UART_CMD_ACK_WAIT;
  if (CmdAckFun_8 == TxWaitForStat)
  {
        if(UartRxBuf[4] != 0x13){
             P_RxBufWrite=0;       //reset P_RxBufWrite
             return UART_CMD_ACK_ERR;
        }
  }
  else
  {
           if(CmdAckFun_8 == TxSetNetCmdAckCode)
          {
              printf("[Eden]->cmd=0x%x recv_4 =0x%x\r\n",CmdAckFun_8,UartRxBuf[4]);
              if ( (0x14 != UartRxBuf[4])/*||( UartRxBuf[6] != 3)*/)
                {
                    P_RxBufWrite=0;       //reset P_RxBufWrite
                    return UART_CMD_ACK_ERR;
                }
          }
          else if(CmdAckFun_8 == TxFactorTestCmdAckCode)
          {
              if ( ((UartRxBuf[6]) != 2)||(0x13!=UartRxBuf[4]) )
                {
                    P_RxBufWrite=0;       //reset P_RxBufWrite
                    return UART_CMD_ACK_ERR;
                }     
          }
          else if((CmdAckFun_8!=UartRxBuf[4])|(fg_RXFrame==FRAME_ERROR))//判断是否是当前要ACK,ACK是否错误码
          {
                fg_RXFrame=ZERO_FRAME;       //reset fg_RXFrame信号
                P_RxBufWrite=0;             //reset P_RxBufWrite
                return UART_CMD_ACK_ERR;
          }  
        
  }
  printf("[Eden]UART_CMD_ACK_OK,cmd=0x%x\r\n",CmdAckFun_8);
   return UART_CMD_ACK_OK;
}




uint8_t AppInstalledFlag=1;//=0,设备处于出厂状态，1=表示已安装APP，可工作,默认


U16 voltage_adc[] = 
{
    /*0% 5%   10%  15%  20%*/
    2739,2822,2859,2887,2909,
    /*25% 30%  35%  40%  45%*/
    2928,2944,2959,2975,2993,
    /*50% 55%  60%  65%  70%*/
    3013,3038,3069,3104,3145,
    /*75% 80%  85%  90%   95%*/
    3189,3233,3260,3320,3347,
    /*100%*/
    3390
};

U16 charge_voltage_adc[] = 
{
    /*0% 5%   10%  15%  20%*/
    2739,2822,3009,3050,3100,
    /*25% 30%  35%  40%  45%*/
    3200,3300,3350,3395,3440,
    /*50% 55%  60%  65%  70%*/
    3470,3485,3500,3510,3528,
    /*75% 80%  85%  90%   95%*/
    3540,3558,3575,3590,3600,
    /*100%*/
    3628
};
static U8  now_electricity = 0;
static U8  last_electricity = 0;

void calc_electricity(U16  voltage)
{
    U8 i;
    last_electricity = now_electricity;
    //sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
    for(i=0;i<21;i++)
    {
        if(sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)
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
    if(i == 0)
    {
        now_electricity = 0;
    }
    else if(i == 21)
    {
        now_electricity = 100;
    }
    else
    {
        now_electricity = 5*(i-1);
    }

    if(((now_electricity < last_electricity) && (sys_state_info_p.robot_state == ROBOT_STATE_CHARGING))
        || ((now_electricity > last_electricity) && (sys_state_info_p.robot_state != ROBOT_STATE_CHARGING)))
    {
        if(last_electricity > 0)
        {
            now_electricity = last_electricity;
        }
    }

    if(sys_state_info_p.charging_state == CHARGING_COMPLETE)
    {
        now_electricity = 100;
    }

    
}
U8 get_bat_level(void)
{
    return now_electricity;
}

/*****************************************************
**函数体名：WifiData_Period_Process
**功能描述：Wifi模块数据处理
**输入参数：无
**输出参数：无 
*****************************************************/

uint8_t ExtCmdInserted=DefaultIdle; //触发WifiCard 状态机工作,命令                                    

uint8_t WorkingStatus=DefaultIdle;  //状态机
uint8_t NextWorkingStatus=DefaultIdle;   //目标态

uint8_t WaitingRxPackCounter=0;        //1000mS=20mS*50
uint8_t RxPackTimeOutErrorCounter=0;    //状态机超时错计数
uint8_t OldSystem20msSig=0;             //20ms 系统时基旧值
uint8_t flag=1;//debug
uint8_t MapDatFlag=OFF;
uint8_t MapFunFlag=OFF;
U8      MapDatCount=0;
extern uint8_t  Sim_oData4[1];

U16 convert_to_adc( s16 voltage) 
{
    U32 mv = voltage<<12;
    mv = mv/18975;//0.18 ---> 18032; 0.174 --> 18975       
    return (U16)mv;
}

void WifiData_Period_Process(void)
{
  
  uint8_t UartCmdAck;
  static U16  voltage_average = 0;
  U16   tmp_voltage = 0;
  static U16   tmp_cnt = 0;
  static U8   first_check_voltage_flag = 0;
  sys_info_get(SYS_STATE_INFO, (long )&sys_state_info_p);
  sys_info_get(SYS_CURRENT_INFO,(long)&sys_current_info_p);

  tmp_voltage = convert_to_adc(sys_current_info_p.battery_voltage);

  voltage_average += tmp_voltage; 

 
    if(voltage_average >= 5000)
    {
	    voltage_average = (voltage_average>>1);
    }

    tmp_cnt += 1;
    if((tmp_cnt >= 2000)||((first_check_voltage_flag == 0)&&(tmp_cnt == 20)))
    {
        calc_electricity(voltage_average);
        tmp_cnt = 0;
        first_check_voltage_flag = 1;
    }

    if(wifi_send_sleep_flag == 1)
    {
       // return;
    }
 // if(OldSystem20msSig != Fg_System20msSig) //计数--
 // {
      //OldSystem20msSig=Fg_System20msSig;
      if(WaitingRxPackCounter) WaitingRxPackCounter--;
  //}
/*
#ifndef FULL_FAST_RUN
  else            //20ms 执行一次，信号读写
        return ;        
#endif
*/
  if(timer_elapsed(wifi_delay_ts) > 130)
  {
    if(MapDatCount > 0)
    {
      MapDatCount--;
    }
    
    wifi_delay_ts = timer_ms();
  }

  //在进入这个函数之前，应该先判断一次是否有下行的控制命令，如果没有下行的控制命令再判断其他三种指令
      
  
  switch(WorkingStatus)
   {
  
//*****************WaitRX 中间状态    
       case WaitRX:
          switch(NextWorkingStatus)
          {
              case S_WAIT_DEL_BOND_ACK:
              case S_WAIT_TX_MAP_DAT_ACK:                   //等待地图数据发送后的ACK
              case S_WAIT_TX_STAT_DAT_ACK:               //等待状态数据发送后的ACK ,都一样的
                   UartCmdAck = ReadData_From_UartRxBuf(TxMapDatAckCode);
                   break;
              case S_WAIT_FACTOR_TEST_ACK:
                    UartCmdAck = ReadData_From_UartRxBuf( TxFactorTestCmdAckCode );
                    break;
              case S_WAIT_SET_NET_ACK:
                    UartCmdAck = ReadData_From_UartRxBuf(TxSetNetCmdAckCode);
                    break;
              case S_WAIT_RECV_STAT:
                   UartCmdAck = ReadData_From_UartRxBuf(TxWaitForStat);
                   break;
              default: 
                    WorkingStatus=DefaultIdle;
                    NextWorkingStatus=DefaultIdle;   
                    break;   
          }      
          
          if (UartCmdAck==UART_CMD_ACK_OK)           //find ok 
          { 
            //P_RxBufWrite = 0; //Eden
            RxPackTimeOutErrorCounter=0;
            WaitingRxPackCounter=0;
            WorkingStatus = NextWorkingStatus;         //jump下个状态
            NextWorkingStatus =DefaultIdle;           //恢复等待状
            flag=1;//debug
          }
          else if((UartCmdAck==UART_CMD_ACK_WAIT)&&(WaitingRxPackCounter!=0))
          {  
               if((WaitingRxPackCounter<WAITING_TIMEOUT_COUNTER__MAXVAL-10)&&(flag==1))//debug
               {
                      flag=0;
                      WaitingRxPackCounter=WaitingRxPackCounter;}
          }
          else         
         {  
                  RxPackTimeOutErrorCounter++;        //计出错次数                                              
                  WorkingStatus = WaitRX;              
                  WaitingRxPackCounter=0;   
                  ResetUartRxBuf();                  //reset rxbuf
                  if(RxPackTimeOutErrorCounter==200) //Wificard 死机自动复位 10*0.8S=8S
                  {
                     RxPackTimeOutErrorCounter=0;
                     WorkingStatus =  PinResetWifiCard;
                  }   
          }  
          break; 
//************ PinReset WifiCard         
      case PinResetWifiCard: 
          //UART_Config_PinResetWifiCard();  //cold 启动，[保护性重新启动],NeedTest
          WorkingStatus =  DefaultIdle;
          NextWorkingStatus = DefaultIdle; 
          ResetUartRxBuf();                 //reset rxbuf    
          break;        

      case DefaultIdle:
           //
           if( ExtCmdInserted != 0x00)
           {
               NextWorkingStatus = ExtCmdInserted;
               WorkingStatus = NextWorkingStatus;
              
           }
           
            ExtCmdInserted = 0x00;
        break;
        
       case S_TX_MAP_DAT://TxMapDat:
              //发送地图数据
             //uint8_t mapDatBuff[6*n] = {0};
            //m_mapData.datalen = n*6;
            //m_mapData.p_buff = mapDatBuff;
            //sendMapDat(m_mapData);
            WorkingStatus = WaitRX;                       //jump等待 ack
            NextWorkingStatus =S_WAIT_TX_MAP_DAT_ACK;          
        break;
        
       case S_TX_STAT_DAT://TxStatDat:
            
            sendStatDat((uint8_t *)&m_statDat);
            WorkingStatus = WaitRX;                       //jump等待 ack
            NextWorkingStatus =S_WAIT_TX_STAT_DAT_ACK;  
        break;
           
       case S_TX_SET_NET://TxSetNet:
            sendSetNetCmd();
            WorkingStatus = WaitRX;                       //jump等待 ack
            NextWorkingStatus =S_WAIT_SET_NET_ACK;  
        break;
        
       case S_TX_FACTOR_TEST://TxFactorTest:
           sendFactorCmd();
           WorkingStatus = WaitRX;                       //jump等待 ack
           NextWorkingStatus =S_WAIT_FACTOR_TEST_ACK;  
        break;

       case S_TX_DEL_BOND://TxDelBond:
           sendDelBondCmd();
           WorkingStatus = WaitRX;                       //jump等待 ack
           NextWorkingStatus =S_WAIT_DEL_BOND_ACK;  
        break;
  
       case S_TX_MODULE_STAT:                           //其实这里并没有发什么，只是找个途径查询状态而已
           WorkingStatus = WaitRX;                      
           NextWorkingStatus = S_WAIT_RECV_STAT; 
       break;    
     default:
          WorkingStatus=DefaultIdle; 
          break;        
   } 

  
}







void set_wifi_enable(uint8_t value)
{
	
	if(value)
	{
		wifi_enable_flag=1;
        gpio_set_value(AM_IO_WIFI_POWER,1);
	}
	else
	{
		wifi_enable_flag=0;
        gpio_set_value(AM_IO_WIFI_POWER,0);
	}
	
	
}
/*
void init_wifi_uart(void)
{
    set_wifi_enable(1);
    wifi_delay_ts = allwork_timer_ms();
    while(allwork_timer_elapsed(wifi_delay_ts) < 3);
    UART_Config_PinResetWifiCard();
    wifi_delay_ts = allwork_timer_ms();
}*/
extern int gpio_direction_output(unsigned gpio, int value);
void init_wifi_module(void)
{
    gpio_request_one(AM_IO_WIFI_POWER,GPIO_F_DIR_OUT|GPIO_F_INIT_LOW);
    gpio_request_one(AM_IO_RESET_WIFI,GPIO_F_DIR_OUT|GPIO_F_INIT_LOW);
    gpio_set_value(AM_IO_WIFI_POWER,1);
    gpio_set_value(AM_IO_RESET_WIFI,0);
}

void powerup_wifi_module(void)
{
    gpio_set_value(AM_IO_WIFI_POWER,1);
    gpio_set_value(AM_IO_RESET_WIFI,1);
  //  gpio_set_value(GPIOD(12), 1);
 //   gpio_set_value(GPIOD(13), 1);
   // gpio_set_value(GPIOD(9), 1);
 
}

uint8_t get_wifi_enable_state(void)
{
    return wifi_enable_flag;
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
   // uartwifi_init();
   
    set_wifi_enable(0);
    
}
uint8_t InsertExtCmd(uint8_t ExtCmd)
{
   uint8_t status=SUCCESS_OK;
   	
  if(ExtCmdInserted==DefaultIdle)    //可接受外部命令
  {
     ExtCmdInserted = ExtCmd;        //reset 外部命令
     printf("reset wifi cmd!!\r\n");
     //songplayer_play_id(SONG_ID_WIFI_CONNET_OK, 0);    
  }
  else  
  {
     status=NO_SUCCESS;
     //songplayer_play_id(SONG_ID_WIFI_CONNECT_fail, 0);
  }
  return  status;
}

/*
*   calCheckSum()
*
*   Eden
*/
uint16_t calCheckSum(uint8_t * p_data, uint16_t len)
{
    int i;
    uint16_t  n;
    unsigned short c = 0;
    i = 4;
    n = p_data[2]*256 + p_data[3]-2; //len
    while(n>1){
        c += (((uint8_t)p_data[i]<<8) | ((uint8_t)p_data[i+1]));
        c = c & 0xffff;
        n -= 2;
        i += 2;
    }
    if (n>0) c = c^(unsigned short)((uint8_t)p_data[i]);
    return c;
}

#endif

