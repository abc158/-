
#ifndef __DEFTYPE_SIMSWEEP_H__
#define __DEFTYPE_SIMSWEEP_H__
//*************************
#define Map_Sim_BlockNum_C 4
extern uint8_t Status_WifiDevice;
extern uint8_t   Map_Sim_Get_Pos;
extern uint16_t  Map_Sim_CurPointer;
extern uint16_t  Map_Sim_HighIndex;
extern uint16_t  Map_Sim_BlockNum;
extern U32 request_map_points(U8 msg_len);
//**********************
void Action_0X00(uint8_t AppKeySig0X00);  
void Action_0X01(uint8_t AppKeySig0X01);  
void Action_0X02(uint8_t AppKeySig0X02);  
void Action_0X03(uint8_t AppKeySig0X03); 
void Action_0X04(uint8_t AppKeySig0X04);
void Action_0X05(uint8_t AppKeySig0X05);
//****************1V3_Begin  ?±????,????????
void Action_0X1A(uint8_t AppKeySig0X1A);
//****************1V3_End
void Action_0X08_0X17(uint8_t *pAppointBuf,uint8_t len);  
void Action_0X18_0X19(uint16_t OEMCode);  
void Action_0X1B_0X1E(uint8_t *pAppointBuf);
void Action_0X20_0X21(uint8_t *pAppointBuf,uint8_t len);
void Action_0X20_0X25(uint8_t *pAppointBuf,uint8_t len);
void Action_0XF1(uint8_t WifiCardStatusReg);  
void Action_0XF8_0XFF(uint8_t *pRTCBuf,uint8_t len,uint8_t APPinstalledEvent);  
void DayDayADD1(void);//º”“ªÃÏ
void DayDaySub1(void);//ºı“ªÃÏ
uint8_t GetMaxDayM(uint8_t mon_8t);//»°ƒ≥‘¬µƒ◊Ó¥ÛÃÏ ˝
//**********************
void MovUp(void);//œÚ«∞,
void MovLeft(void);//œÚ◊Û,
void MovRight(void);//œÚ”“,
void DownVolume(void);//ºı“Ù¡ø
void UpVolume(void);//º”“Ù¡ø
//********************
void ChargeMode(void);
void  Get_oData0_0X50_0X53(uint8_t *p_oData0Buf,uint8_t BufByteLen);
void  Get_oData1_0X58_0X5F(uint8_t *p_oData1Buf,uint8_t BufByteLen);   
void  Get_oData2_0X60_0X67(uint8_t *p_oData2Buf,uint8_t BufByteLen); 
void  Get_oData3_0X68_0X6B(uint8_t *p_oData3Buf,uint8_t BufByteLen);
//void  Get_oData6_0X80_0X87(uint8_t *p_oData6Buf,uint8_t BufByteLen);
//void  Get_oData7_0X88_0X8F(uint8_t *p_oData7Buf,uint8_t BufByteLen);
//void  Get_oData8_0X90_0X97(uint8_t *p_oData8Buf,uint8_t BufByteLen);
//void  Get_oData9_0X98_0X9F(uint8_t *p_oData9Buf,uint8_t BufByteLen);
//***********1V2_Begin
void  Get_oData4_0X54_0X54(uint8_t *p_oData4Buf,uint8_t BufByteLen);
void  Get_oData5_0X55_0X55(uint8_t *p_oData5Buf,uint8_t BufByteLen);
void  Get_oData6_0X70_0X77(uint8_t *p_oData6Buf,uint8_t BufByteLen);
void  Get_oData7_0X78_0X7F(uint8_t *p_oData7Buf,uint8_t BufByteLen);
void  Get_oData8_0X80_0X87(uint8_t *p_oData8Buf,uint8_t BufByteLen);
void  Get_oData9_0X88_0X8F(uint8_t *p_oData9Buf,uint8_t BufByteLen);
void  Get_oData10_0X90_0X97(uint8_t *p_oData10Buf,uint8_t BufByteLen);
void  Get_oData11_0X98_0X9F(uint8_t *p_oData11Buf,uint8_t BufByteLen);
void  Get_oData12_0XA0_0XA7(uint8_t *p_oData12Buf,uint8_t BufByteLen);
void  Get_oData13_0XA8_0XAF(uint8_t *p_oData13Buf,uint8_t BufByteLen);
void  Get_oData14_0XB0_0XB7(uint8_t *p_oData14Buf,uint8_t BufByteLen);
void  Get_oData15_0XB8_0XBF(uint8_t *p_oData15Buf,uint8_t BufByteLen);
void  Get_oData16_0XC0_0XC7(uint8_t *p_oData16Buf,uint8_t BufByteLen);
void  Get_oData17_0XC8_0XCF(uint8_t *p_oData17Buf,uint8_t BufByteLen);
void ui_put_map_point_info(uint16_t x, uint16_t y, uint8_t type, uint16_t direction);

#define MAP_POINT_BUFFER_SIZE 16
#define MAP_FRAME_RETRY_COUNT 5
typedef struct
{
    map_point_t points[MAP_POINT_BUFFER_SIZE];
    uint16_t direction;
    uint8_t count;
} real_map_points_t;

U8 *get_his_map_buffer(void);
//***********1V2_End
//***********
void Map_Sim_Process(void);
void Map_Sim_Get_Process(void);
void clear_map_index(void);
uint16_t  Get_OEM_CODE(void);//??OEM CODE
#endif