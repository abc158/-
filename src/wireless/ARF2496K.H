/**************************************************************
########有限公司
文件名：drv_ARF2498.h
版本：1.0
作者：胡忠伟 lee
时间：2015.01.01
修改信息：无
**************************************************************/
#ifndef  __ARF2498_H
#define  __ARF2498_H

#define ARF2496K_CHANNEL  0x44  //通道数
#define DUIMA_CHANNEL  0x44  //通道数

#define HOST_ADDR 0x02  // RF主机地址
#define SLAVE_ADDR 0x01  // RF从机地址

#define HOST_ID 0  // RF主机编号
#define REMOTE_ID 1  // RF从机编号
#define DOCK_ID 2  // RF从机编号
#define VWALL_ID 3  // RF从机编号

//下面定义需要用到的定义
#define ARF2498_STMODE	 	0X00 	//芯片在待机模式
#define ARF2498_RXMODE	 	0X80 	//芯片在接收模式
#define ARF2498_TXMODE	 	0XA0   //芯片在发送模式
#define ARF2498_TXACKMODE	 	0XC0   //芯片在发送，应答模式
#define ARF2498_REUSETX 		0XD0   //重新发送

#define ARF2498_CHANNEL  		0x44 	//通道数  2440 MHZ

#define RFREG_Rx_Chn  0x00
#define RFREG_State  0x01
#define RFREG_Int  0x02
#define RFREG_PWR  0x23
#define RF_IRQ_CLR  0x3f

#define  RF_STANDBY	0	// 待机模式
#define  RF_SLEEP		1	// 睡眠模式
#define  RF_SEND		2	// 发送模式
#define  RF_RECV		3	// 接收模式

typedef struct{
	u8 mode;			// 工作模式
	u8 resend_en;	// 重复发送使能
	u8 ask_en;		// 发送完是否允许接收应答
}RF_DATA;

extern void  ReadDeviceAddress(void);
extern bool QueryARF2496KInceptSign(void);  //查询无线芯片是否收到数据，收到则返回TRUE，否则返回FALSE。读出数据后将标志清除
extern void ReadRF(u8 *readdata,u8 readnumber);//从缓存中读出RF数据，最大为16字节，
extern void SendRF(u8 *senddata,u8 sendnumber);//发送数据，最大为16字节
extern void SetPartnershipAddr(void);//设置为配对地址
extern void DisableRFIncept(void);//芯片进入待机状态，不允许无线接收
extern void EnableRFIncept(void);//使能无线接收
extern void InitARF2496k(void);//初始化无线模块芯片
extern void RFIrq(void);
extern void EnterRXMode(void);
extern void Set_rfchannel(u8 ch);
extern void SetDestinationAddress(void);	//设置为目标地址
extern void PartnershipRF(void); //发送对码信息
extern void Syn_DockRF(void) ;

#endif
