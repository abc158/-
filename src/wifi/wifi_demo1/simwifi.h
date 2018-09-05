
#ifndef __DEFTYPE_SIMWIFI_H__
#define __DEFTYPE_SIMWIFI_H__

//#define  gzsdebug                   //?��?��?��??
//#define  gzsdebugOut
#define  FULL_FAST_RUN               //������壬�����ܿ��ٶ�ȡ������ӦAPP�����ź�,����20mSһ�ζ�ȡ
#define  UART_BAUD_RATE     38400  //9600 //���ڲ����� 
#define  APP_COM_NUM       0X03EC //һ��������Ŀ(for һ�ֵ����豸)===�е���APP�����,
                                   //����������ƽ̨����
#define  WAITING_TIMEOUT_COUNTER__MAXVAL        40      //40*20ms=800ms==0.8S �ȴ�NEXT WORKING ������
//#define  WAITING_TIMEOUT_COUNTER__MAXVAL          80
//****************************************************************
#define  UART_RX_BUF_SIZE   64     //UART���ջ�������С,ͳһ����
#define  UART_TX_BUF_SIZE   64     //UART���ͻ�������С,ͳһ����
//***********************************

#define NO_SUCCESS   0
#define SUCCESS_OK   1
//*******************************
#define WIFI_DEVICE_OK           00 //�豸�������� 
#define IN_RESTORE_FACTORY       03 //wifiģ��,���ڻָ�����������
//*******************************
#define ZERO_FRAME       0
#define FRAME_OK         1
#define FRAME_ERROR      2
#define UART_CMD_ACK_OK           0     //ack oK
#define UART_CMD_ACK_WAIT         1     //ACK WAIT
#define UART_CMD_ACK_ERR          2     //ACK ERROR
//*******
#define  WaitRX                                 0X01   //����״

#define  DefaultIdle                            0X00   //����״
#define  PinResetWifiCard                       0X02   //ResetPin,��λWifiCard


//*********************************************

//--Eden code start------

#define S_WAIT_DEL_BOND_ACK 3
#define S_WAIT_TX_MAP_DAT_ACK 4
#define S_WAIT_TX_STAT_DAT_ACK 5
#define S_WAIT_FACTOR_TEST_ACK 6
#define S_WAIT_SET_NET_ACK 7
#define S_WAIT_RECV_STAT 13



#define S_TX_DEL_BOND 8
#define S_TX_MAP_DAT  9
#define S_TX_STAT_DAT 10
#define S_TX_FACTOR_TEST 11
#define S_TX_SET_NET 12
#define S_TX_MODULE_STAT 14






#define TxMapDatAckCode         0x14              //�ȴ���ͼ���ݷ��ͺ��ACK

#define TxStatDatAckCode 0x14//�ȴ�״̬���ݷ��ͺ��ACK
#define TxFactorTestCmdAckCode   0x33
#define TxSetNetCmdAckCode        0X13
#define TxWaitForStat            0x23

#define TxMapDatFuncCode         0x02       //�ȴ���ͼ���ݷ��ͺ��ACK

#define TxStatDatFuncCode        0x01       //�ȴ�״̬���ݷ��ͺ��ACK
#define TxFactorTestCmdFuncCode:   0x03
#define TxSetNetCmdFuncCode        0X03     

//״̬������




//--Eden code end --------


//#define  NullTestAckCode       (NullTestFunCode|0X80)
//#define  ReadMemAckCode        (ReadMemFunCode|0X80)
//#define  WriteMemAckCode       (WriteMemFunCode|0X80)
//#define  WriteMemEEROMAckCode  (WriteMemEEROMFunCode|0X80)
//#define  EnReadMemAckCode      (EnReadMemFunCode|0X80)
//#define  EnWriteMemAckCode     (EnWriteMemFunCode|0X80)
//#define  WReadMemAckCode       (WReadMemFunCode|0X80)
//#define  WriteRMemAckCode      (WriteRMemFunCode|0X80)

#define  UartCmdFinOK          0X00   //=0X00,�������
#define  UartCmdFinErr1        0X01   //=0X01,������Ч��
#define  UartCmdFinErr3        0X03   //=0X03,����δ֪��
#ifdef USE_WIFI_DEMO_1
#define UART_CMD_MAX 13
#endif

							


/*!< Signed integer types  */



#define oData0Mode                0 //ȱʡloopģʽ
#define oData1Mode                1 //����Ƭ1,���
#define oData2Mode                2 //����Ƭ2,���
#define oData3Mode                3 //����Ƭ3,���
extern uint8_t DataModeFlag;

#define MapDatDelay 3
#define ONCE   2 


//**********************
extern uint8_t ExtCmdInserted; 
extern uint8_t  wifi_send_sleep_flag;
//**********************
uint8_t InsertExtCmd(uint8_t ExtCmd);
extern U16 get_uart_cmd(void);
//extern key_state_t *get_uart_cmd_state(void);
extern void uart_cmd_rounte(void);
extern u16 sys_uart_trig_tx_evnet(u16 uart_index);
extern void WifiData_Period_Process(void);
extern void UART_Config_PinResetWifiCard(void);
extern void IRQ_Usart1RxData_Process(uint8_t value);
extern void set_wifi_enable(uint8_t value);
extern uint8_t get_wifi_enable_state(void);
extern void set_reset_wifi_flag(uint8_t value);
extern uint8_t get_reset_wifi_flag(void);
extern void exit_wifi(void);
extern void init_wifi_module(void);
extern void powerup_wifi_module(void);
extern U8 get_bat_level(void);
extern uint16_t calCheckSum(uint8_t * p_data, uint16_t len);
//********************

//**********************

//***********************
//***********

#endif


































