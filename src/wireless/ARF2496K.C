//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/////////下面是芯片的IO口定义/////////////////////////////////

//#define MCU_ID 0X1FFFF7E8  // STM32 ID起始地址，共96bit
#define MCU_ID 0X1FFFF7E2  // STM32 ID起始地址，共96bit
#define PARTNERSHIP  0Xff

////////////////////////头文件////////////////////////////////////
#include "syscall_api.h"
#include "am_exti_cfg.h"
#include "am_device.h"
#include "ui-ir.h"
#include "ARF2496K.H"
#include "ui-config.H"
#include "act.h"
#include "syscall_api.h"
#include "docking-new.h"

#ifdef IR_WIRELESS

////////////////////////全局变量//////////////////////////////////
RF_DATA rf_data;	// RF数据
u8 rssi_val[10];	// 信号强度
u8 rssi_cnt;
u8 pipe_numb;  // 发送端地址
u8 rf_recvbuff[16];
u8 rf_sendbuff[16]={0x98,0x76,0x54,0x32,0x76,0x54,0x32,0x76,0x54,0x32,0x76,0x54,0x54,0x32,0x76,0x54};
u8 rf_askbuff[5] = {0,0,0,0,0};

extern u8 get_sleep_level(void);
extern int hal_id_to_gpio_num(int id);

static volatile bool RFInceptSign;//接收信号，为true表示接收到一个信号，在执行读出函数时将其清掉。

static const u8 PartnershipAddress[6] ={0x98,0x76,0x54,0x32,HOST_ADDR,SLAVE_ADDR};
static u8 RFGroupAddress[4] = {0x98,0x76,0x54,0x32};//器件地址
static u8 RFDeviceAddress = HOST_ADDR;//器件地址
static u8 RFSlaveAddr = SLAVE_ADDR;//分机地址
static unsigned char rf_channel = 0x44; // 通讯频道

static volatile bool EnableRFState;//是否使能无线接收。在芯片进行一些操作后自动进入接收模式或者进入待机模式。
u8 check_partnership_ok_flag = 0;
u8 signal_flag=0;
U32 ARF2496k_id = 0;

int arf2498_spi_id = 0;


extern int gpio_direction_input(unsigned gpio);
extern int gpio_direction_output(unsigned gpio, int value);
extern u16 system_spi_tx_fifo_full(void);
extern u16 system_spi_rx_fifo_empty(void);
extern uint8_t remote_ir_get(IR_REMOT_POSITION_E index);
s16 afio_mutex_mod_get(u32 id);
void ir_rx_code_get(u8 chan, u8 val);

const static u8  RF_InitTable[] =
{
  0x00,ARF2498_CHANNEL,
  0x03,0x6f,		//  组地址4个字节 ,0DB, 2byte-CRC
  0x04,0xff,		//  运行所有通道
  0x05,0x00,		//  禁止所有通道自动应答
  0x06,0x00,		//  禁止所有通道动态负载
  0x07,0x11,		//  设置重复发送等待时间 500us+250us*Resend_cnt
  //	0x08,0x30,		//  自动重复发送3次
  0x08,0x30,		//  自动重复发送3次
  0x16,0x10,		//   负载大小，16个字节
  0x17,0x10,		//
  0x18,0x10,		//
  0x19,0x10,		//
  0x1a,0x10,		//   负载大小，16个字节
  0x1b,0x10,		//
  0x1c,0x10,		//
  0x1d,0x10,		//

  //0x20,0x20,		//  使能自动刷新信号强度
  0x21,0x04,		//  LNA高增益
  0xaa,0xaa,
};
/////////////////////////////////////////////////////////////
//延时程序
static void delay(u32 dl)
{
	 /*
  dl = dl*3;
  for(; dl != 0; dl--);
  */
  u16 i;
  for(i=0;i<0xD56;i++)//1us
  {
    ;
  }
}
static void delay_us(u16 us)
{
   u16 i;
   for(i=0;i<us;i++)
   delay(1);
}
static void delay_ms(u8 ms)
{
  delay_us(ms*1000);
}

//置1
void SetARF2498_MISO_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 1);
  //gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 1);
   gpio_set_value(AM_I0_FLASH_DATA_O, 1);
}
//清0
void ResetARF2498_MISO_Port(void)
{
 // gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 0);
 // gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 0);
 gpio_set_value(AM_I0_FLASH_DATA_O, 0);
}

//置1
void SetARF2498_MOSI_Port(void)
{
 //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 1);
 // gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 1);
   gpio_set_value(AM_I0_FLASH_DATA_I, 1);
 // gpio_direction_output(GPIOF(14), 1);
  //i0_hal_digital_io_set(GPIOF(14) ,1);
}
//清0
void ResetARF2498_MOSI_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 0);
  //gpio_set_value(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_O), 0);
  gpio_set_value(AM_I0_FLASH_DATA_I, 0);
  // gpio_direction_output(GPIOF(14), 0);
//i0_hal_digital_io_set(GPIOF(14) ,0);

}

//置1
void SetARF2498_SCK_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_CLOCK), 1);
  gpio_set_value(AM_I0_FLASH_CLOCK, 1);
}
//清0
void ResetARF2498_SCK_Port(void)
{
// gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_CLOCK), 0);
  gpio_set_value(AM_I0_FLASH_CLOCK, 0);
}

//置1
void SetARF2498_CS_Port(void)
{
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 1);
  //gpio_set_value(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 1);
  gpio_set_value(AM_IO_ARF2498_SPI_EN, 1);
}
//清0
void ResetARF2498_CS_Port(void)
{
 //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 0);
 // gpio_set_value(hal_id_to_gpio_num(HAL_I0_WIRELESS_SPIEN), 0);
 gpio_set_value(AM_IO_ARF2498_SPI_EN, 0);

}



//置1
void SetARF2498_CE_Port(void)
{
//  gpio_direction_output(hal_id_to_gpio_num(HAL_I0_LED_DIG1), 1);
   gpio_set_value(AM_I0_WIRELESS_SPIEN, 1);
}
//清0
void ResetARF2498_CE_Port(void)
{
 // gpio_direction_output(hal_id_to_gpio_num(HAL_I0_LED_DIG1), 0);
    gpio_set_value(AM_I0_WIRELESS_SPIEN, 0);
}

u8 ReadARF2498_MISO_Port(void)
{
  return gpio_get_value(AM_I0_FLASH_DATA_O);
}
/***********************************************************
初始化RF硬件的IO口
***********************************************************/
void InitRfHardWare(void)
{
#ifdef WIRELESS_SIMULATE_SPI

  arf2498_spi_id = spi0_id_get();

  irq_io_set_vaule(AM_I0_FLASH_DATA_I, 0);
  irq_io_set_vaule(AM_I0_FLASH_CLOCK, 0);
  //gpio_direction_output(hal_id_to_gpio_num(HAL_I0_FLASH_DATA_I), 0);
  irq_io_set_vaule(AM_I0_FLASH_DATA_O)  ;

  irq_io_set_vaule(AM_I0_WIRELESS_SPIEN, 0);
  irq_io_set_vaule(AM_I0_LED_DIG1, 0);

  irq_io_set_vaule(AM_IO_ARF2498_IRQ);//ARF2496K_TRINT


  ResetARF2498_SCK_Port();
  ResetARF2498_CE_Port();
  SetARF2498_MOSI_Port();
  SetARF2498_CS_Port();
#else

  gpio_set_value(AM_IO_ARF2498_SPI_EN, 0);
  gpio_set_value(AM_I0_WIRELESS_SPIEN, 0);

  //gpio_direction_input(HAL_IO_ARF2498_IRQ);//ARF2496K_TRINT

  ResetARF2498_CE_Port();
  SetARF2498_CS_Port();
#endif

}

#ifndef WIRELESS_SIMULATE_SPI

static u16 spi_ARF2498_transmit_byte(u8 data)
{
    volatile u8 arg = 0;
    ioctl(arf2498_spi_id,SPI_FIFO_TX_STATE,(void*)&arg);
    //while(system_spi_tx_fifo_full());
    while(arg)
    {
      ioctl(arf2498_spi_id,SPI_FIFO_TX_STATE,(void*)&arg);
    }
    //system_spi_write_byte(data);
    write(arf2498_spi_id,(u8 *)&data,1);

    //while(system_spi_rx_fifo_empty());

    ioctl(arf2498_spi_id,SPI_FIFO_RX_STATE,(void*)&arg);
    while(arg)
    {
      ioctl(arf2498_spi_id,SPI_FIFO_RX_STATE,(void*)&arg);
    }
    read(arf2498_spi_id,(u8*)&arg,1);
    //return system_spi_read_byte();
    return arg;
}
#endif

/************************************************************
读一个字节,从高位开始到低位读
************************************************************/
u8 SPI_RW(u8 dat)
{
  u8 i;
  //GPIO_ResetBits(ARF2498_SCK_Port,ARF2498_SCK_Pin);
  ResetARF2498_SCK_Port();
  delay_us(1);
  for(i=0; i<8; i++)
  {
    if(dat&0x80){
      SetARF2498_MOSI_Port();
      //GPIO_SetBits(ARF2498_MOSI_Port, ARF2498_MOSI_Pin);		//拉高数据线
    }
    else{
      ResetARF2498_MOSI_Port();
      //GPIO_ResetBits(ARF2498_MOSI_Port, ARF2498_MOSI_Pin);    //拉低数据线
    }
    dat <<= 1;
    SetARF2498_SCK_Port();
    //GPIO_SetBits(ARF2498_SCK_Port, ARF2498_SCK_Pin);
    delay_us(1);
    //if(GPIO_ReadInputDataBit(ARF2498_MISO_Port,ARF2498_MISO_Pin)){
    if(ReadARF2498_MISO_Port()){
      dat |= 0x01;
    }
    else{
      dat &= 0xfe;
    }
    ResetARF2498_SCK_Port();
    //GPIO_ResetBits(ARF2498_SCK_Port,ARF2498_SCK_Pin);
    delay_us(1);
  }
  return dat;
}
/************************************************************
使用spi接口对芯片的一个寄存器进行读操作	 寄存器数不能大于63
************************************************************/
u8 ReadARF2498Reg(u8 addr)
{
  u8 RegValue;  //寄存器值
  u8 commd;
  commd = addr & 0x3f;    //将高位屏蔽,在读SPI时，第7,6位为00，
#ifdef WIRELESS_SIMULATE_SPI
  ResetARF2498_CS_Port();
  delay_us(1);
  //发送8位地址以及读写位
  SPI_RW(commd);
  RegValue = SPI_RW(0);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();

  //hal_ints_off();
  ResetARF2498_CS_Port();
  //delay_us(1);
  spi_ARF2498_transmit_byte(commd);
  //RegValue = system_spi_read_byte();
  RegValue = spi_ARF2498_transmit_byte(0);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();
  //hal_ints_on();
#endif


  return RegValue;
}
/************************************************************
使用spi接口对芯片的一个寄存器进行写操作	 寄存器数不能大于63
************************************************************/
void WriteARF2498Reg(u8 addr, u8 RegValue)		//写入寄存器值
{
  u8 commd;
  commd = (addr&0x3f) | 0x40;    //将高位置1,在写SPI时，第7,6位为01，

#ifdef WIRELESS_SIMULATE_SPI
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  //发送8位地址以及读写位
  SPI_RW(commd);
  SPI_RW(RegValue);
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();
  //hal_ints_off();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  //system_spi_write_byte(RegValue);
  spi_ARF2498_transmit_byte(commd);
  spi_ARF2498_transmit_byte(RegValue);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();

  //hal_ints_on();
#endif
}
/************************************************************
使用spi接口对芯片的一个寄存器进行写操作	 寄存器数不能大于63
************************************************************/
void Flush_Tx_Fifo(void)		//写入寄存器值
{
  u8 commd;
  commd = 0xc5;    //将高位置1,在写SPI时，第7,6位为01，
#ifdef WIRELESS_SIMULATE_SPI
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  //发送8位地址以及读写位
  SPI_RW(commd);
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();
#endif
}
/************************************************************
使用spi接口对芯片的一个寄存器进行写操作	 寄存器数不能大于63
************************************************************/
void Flush_Rx_Fifo(void)		//写入寄存器值
{
  u8 commd;
  commd = 0xc4;    //将高位置1,在写SPI时，第7,6位为01，
#ifdef WIRELESS_SIMULATE_SPI
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  //发送8位地址以及读写位
  SPI_RW(commd);
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  //sys_spi_cs_high();
  SetARF2498_CS_Port();
#endif
}
/************************************************************
	接收芯片的FIFO数据
************************************************************/
void Read_Fifo_Data(u8 *datbuff)		//写入寄存器值
{
  u8 readnumber;
  u8 commd;
  u8 temp;
  commd = ARF2498_RXMODE;    //将高位置1,在写SPI时，第7,6位为01，

#ifdef WIRELESS_SIMULATE_SPI

  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  SPI_RW(commd);
  temp = SPI_RW(0);
  rssi_val[rssi_cnt++] = (temp&0xf0)>>4;	// 信号强度
  if(rssi_cnt>=10){
    rssi_cnt = 0;
  }
  pipe_numb = temp&0x0f;		// 发送端地址
  readnumber = SPI_RW(0)&0x1f;	// 数据长度
  for (temp = 0; temp<16; temp++)
  {
    if(temp < readnumber){
      datbuff[temp]=SPI_RW(0);	// 数据包
    }
    else{
      datbuff[temp]=SPI_RW(0);	// 数据包
    }
  }
  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();

#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  temp = spi_ARF2498_transmit_byte(0);//system_spi_read_byte();
  rssi_val[rssi_cnt++] = (temp&0xf0)>>4;	// 信号强度
  if(rssi_cnt>=10){
    rssi_cnt = 0;
  }
  pipe_numb = temp&0x0f;		// 发送端地址

  readnumber = spi_ARF2498_transmit_byte(0)&0x1f;	// 数据长度
  for (temp = 0; temp<16; temp++)
  {
    if(temp < readnumber){
      datbuff[temp]=spi_ARF2498_transmit_byte(0);	// 数据包
    }
    else{
      datbuff[temp]=spi_ARF2498_transmit_byte(0);	// 数据包
    }
  }

  //sys_spi_cs_high();
  SetARF2498_CS_Port();
#endif
}
/************************************************************
	发送芯片的FIFO数据,无应答或等待应答
************************************************************/
void Write_Fifo_Data(u8 rfpipe,u8 nack,u8 *datbuff,u8 sendnumber)		//写入寄存器值
{
  u8 commd;
  u8 cnt;
  commd = ARF2498_TXMODE|0X10|rfpipe;    // 0DB
#ifdef WIRELESS_SIMULATE_SPI
  delay_ms(1);
  //GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  ResetARF2498_CS_Port();
  delay_us(1);
  SPI_RW(commd);
  if(nack == 0){
    SPI_RW(rf_channel);	// 需应答
  }
  else{
    SPI_RW(rf_channel|0x80);	 // 不应答
  }
  for (cnt =0; cnt < sendnumber ; cnt ++)
  {
    SPI_RW(datbuff[cnt]);
  }

  if(sendnumber<16){
    for(cnt = sendnumber;cnt<16;cnt++){
      SPI_RW(rf_sendbuff[cnt]);
    }
  }

  //GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
  SetARF2498_CS_Port();
  //设置 CE 发出50US高脉冲
  //GPIO_SetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
  SetARF2498_CE_Port();
  delay_us(50);
  ResetARF2498_CE_Port();
  //GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
  delay_us(150);  // 需等待130US
#else
  //sys_spi_cs_low();
  ResetARF2498_CS_Port();
  //system_spi_write_byte(commd);
  spi_ARF2498_transmit_byte(commd);
  if(nack == 0){
    spi_ARF2498_transmit_byte(rf_channel);	// 需应答
  }
  else{
    spi_ARF2498_transmit_byte(rf_channel|0x80);	 // 不应答
  }
  for (cnt =0; cnt < sendnumber ; cnt ++)
  {
    spi_ARF2498_transmit_byte(datbuff[cnt]);
  }

  if(sendnumber<16){
    for(cnt = sendnumber;cnt<16;cnt++){
      spi_ARF2498_transmit_byte(datbuff[cnt]);
    }
  }

  //sys_spi_cs_high();
  SetARF2498_CS_Port();

  SetARF2498_CE_Port();
  //delay_us(50);
  ResetARF2498_CE_Port();
  //delay_us(150);  // 需等待130US

#endif
}
/************************************************************
	发送芯片应答包
************************************************************/
void Rf_Send_Ack(u8 rfpipe,u8 *datbuff,u8 readnumber)		//写入寄存器值
{
	u8 commd;
	u8 i = 0 ;
	commd = ARF2498_TXACKMODE|0X08|rfpipe;    // 0DB
	//GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
#ifdef WIRELESS_SIMULATE_SPI
	ResetARF2498_CS_Port();
	SPI_RW(commd);
	for (; i < readnumber ; i ++)
	{
		SPI_RW(datbuff[i]);
	}
	//GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
	SetARF2498_CS_Port();
#else
        //sys_spi_cs_low();
        ResetARF2498_CS_Port();
        spi_ARF2498_transmit_byte(commd);
	for (; i < readnumber ; i ++)
	{
          spi_ARF2498_transmit_byte(datbuff[i]);
	}
        //sys_spi_cs_high();
        SetARF2498_CS_Port();
#endif
	delay_us(150);  // 需等待130US
}
/************************************************************
	发送芯片的FIFO数据,重复发送
*************************************************************/
void Rf_ReSend(u8 rfpipe,u8 *datbuff,u8 readnumber)		//写入寄存器值
{
	u8 commd;
	commd = ARF2498_REUSETX;    // 0DB
	//GPIO_ResetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
#ifdef WIRELESS_SIMULATE_SPI
	ResetARF2498_CS_Port();
	delay_us(1);
	SPI_RW(commd);
	SPI_RW(rf_channel);
    SetARF2498_CS_Port();
#else
        //sys_spi_cs_low();
        ResetARF2498_CS_Port();
        spi_ARF2498_transmit_byte(commd);
        spi_ARF2498_transmit_byte(rf_channel);
        //sys_spi_cs_high();
        SetARF2498_CS_Port();
#endif
	//GPIO_SetBits(ARF2498_CS_Port, ARF2498_CS_Pin);
	delay_us(150);  // 需等待130US
}
/**********************************************************
进入待机模式 26uA // 从省电模式进入需延迟1.5mS
**********************************************************/
void EnterStandMode(void)
{
	//GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
	ResetARF2498_CE_Port();
	rf_data.mode = RF_STANDBY;
}
/**********************************************************
进入待机模式 900nA
**********************************************************/
void RFPowerSaveMode(void)
{
	WriteARF2498Reg(RFREG_PWR,0X02);
	rf_data.mode = RF_SLEEP;
}
/**********************************************************
进入待机模式 15mA
**********************************************************/
void EnterRXMode(void)
{
	Flush_Rx_Fifo();
	WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// 清除所有中断标志
	//GPIO_SetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
	SetARF2498_CE_Port();
   // WriteARF2498Reg(RFREG_Rx_Chn,0xc4);
	rf_data.mode = RF_RECV;
}
/**********************************************************
获取当前RF状态，0 Power_Save,1 standby,,2 tx_ack,3 tx_normal
4 rx_ack,5 rx_normal, 6 rx_receiving,7 invalid
**********************************************************/
u8 RFGetMode(void)
{
	return ReadARF2498Reg(0x22)&0xff;
}

/**********************************************************/
u8 RFGetFIFOState(void)
{
	return ReadARF2498Reg(0x1f)&0xff;
}

u8 RFGetIRQState(void)
{
	return ReadARF2498Reg(RFREG_Int)&0xff;
}

/************************************************************
	使能无线模块中断， //ARF2498_IRQ_Port	//ARF2498_IRQ_Pin
*************************************************************/
void arf2498_handler(u8 p)
{
  RFIrq();
}


void EnableRfInt(void)
{
  //int mod;
  exti_init_t exti_user_init;

  exti_user_init.IntEn       = ENABLE;
  exti_user_init.Int_type    = EXTI_LOW_LEVEL;
  exti_user_init.WakeupirqEn = ENABLE;
  exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;

  ARF2496k_id = open(DEV_EXTI,ARF2496_IRQ_PIN);
  ioctl(ARF2496k_id,EXTI_INIT,&exti_user_init);
  request_irq(ARF2496k_id,(long)arf2498_handler,0);

  // WriteARF2498Reg(RFREG_Int,0X38);	//低电平有效,接收发送都产生中断
  WriteARF2498Reg(RFREG_Int,0X20);	//低电平有效，接收到一个数据包
}
/************************************************************
	使能无线模块中断， //ARF2498_IRQ_Port	//ARF2498_IRQ_Pin
0:
1：
*************************************************************/

void arf2498_sleep(u8 handle_time)
{
  //sys_exti_release(ARF2496k_id);
  if(handle_time == 0)
  {
    exti_init_t exti_user_init;
    exti_user_init.IntEn       = DISABLE;
    exti_user_init.Int_type    = EXTI_LOW_LEVEL;//EXTI_POSITIVE_EDGE;
    exti_user_init.WakeupirqEn = ENABLE;
    exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
    ioctl(ARF2496k_id,EXTI_INIT,&exti_user_init);
    if(get_enter_sleep_state() == SLEEP_MODE_S1)
    {
         EnterRXMode();
    }
  }

  if(handle_time == 1)
  {
    if(get_enter_sleep_state() == SLEEP_MODE_S1)
    {
      SetARF2498_CE_Port();
    }
    else//standby 模式
    {
      EnterStandMode();
    }
  }

}

void arf2498_wakeup(u8 handle_time)
{
  (void)handle_time;
  exti_init_t exti_user_init;
  exti_user_init.IntEn       = ENABLE;
  exti_user_init.Int_type    = EXTI_LOW_LEVEL;//EXTI_POSITIVE_EDGE;
  exti_user_init.WakeupirqEn = ENABLE;
  exti_user_init.Wakeup_type = EXTI_WAKEUP_LOW_LEVEL;
  ioctl(ARF2496k_id,EXTI_INIT,&exti_user_init);
  EnterRXMode();
}

static power_ctrl_t sleep_funcs = {
  .sleep = arf2498_sleep,
  .wake  = arf2498_wakeup,
};
/**********************************************************
使能无线接收
**********************************************************/
void EnableRFIncept(void)
{
  if(FALSE == EnableRFState)
  {
    EnableRFState = TRUE;
	EnterRXMode();
  }
}
/********************************************************
禁止无线接收
********************************************************/
void DisableRFIncept(void)
{
  EnableRFState = FALSE;
  EnterStandMode();
}
/**********************************************************
	读取RF通讯目的地址 ，计算形成本机地址
**********************************************************/
void  ReadDeviceAddress(void)
{
	u32 useraddress=0;
	//u8 temp;
        useraddress = chipid_read();//0x123456789;//
    /*
	for(temp = 0;temp<3;temp++)	{
		//useraddress += *(u32 *)(MCU_ID+temp*4);
		useraddress += (MCU_ID+temp*4);
	}*/
	RFGroupAddress[0] = (u8)(useraddress&0xff);
	RFGroupAddress[1] = (u8)((useraddress>>8)&0xff);
	RFGroupAddress[2] = (u8)((useraddress>>16)&0xff);
	RFGroupAddress[3] = (u8)((useraddress>>24)&0xff);
	//RFDeviceAddress = 0;
	//for(temp = 0;temp<12;temp++)	{
		//RFDeviceAddress += *(u8 *) (MCU_ID+temp);
	//}
}
/**********************************************************
	设置RF的群组地址
**********************************************************/
void SetDestinationAddress(void)
{
	WriteARF2498Reg(0x09,RFGroupAddress[0]);
	WriteARF2498Reg(0x0a,RFGroupAddress[1]);
	WriteARF2498Reg(0x0b,RFGroupAddress[2]);
	WriteARF2498Reg(0x0c,RFGroupAddress[3]);
	WriteARF2498Reg(0x0d,RFDeviceAddress);
	WriteARF2498Reg(0x0e,RFSlaveAddr);
}
/**********************************************************
	设置RF的对码地址
**********************************************************/
void SetPartnershipAddr(void)
{
	WriteARF2498Reg(0x09,PartnershipAddress[0]);
	WriteARF2498Reg(0x0a,PartnershipAddress[1]);
	WriteARF2498Reg(0x0b,PartnershipAddress[2]);
	WriteARF2498Reg(0x0c,PartnershipAddress[3]);
	WriteARF2498Reg(0x0d,PartnershipAddress[4]);	 // 通过此管道对码
	WriteARF2498Reg(0x0e,PartnershipAddress[5]);	 // 通过此管道对码
}
/**********************************************************
	设置RF频道
**********************************************************/
void Set_rfchannel(u8 ch)
{
	if(ch>=125){
		ch = ARF2498_CHANNEL;
	}
	rf_channel = ch;
}
int init_aft2498 = 0;
/**********************************************************
初始化RF
**********************************************************/
void InitARF2496k(void)
{
  //return;//lyy
  static u8 temp = 0;
  InitRfHardWare();
  //delay_ms(10);
  if(ReadARF2498Reg(0x3d) == 0xa1){
    delay_ms(1);  // 读取ID号正确
    //AM_DEBUG_INFO("read ID ok!!!\r\n");
  }

  ReadDeviceAddress();
  Set_rfchannel(ARF2498_CHANNEL);

  while(RF_InitTable[temp] != 0xaa)
  {
    WriteARF2498Reg(RF_InitTable[temp],RF_InitTable[temp+1]);
    delay_us(10);
    if(RF_InitTable[temp+1] != ReadARF2498Reg(RF_InitTable[temp]))
    {
      delay_us(10);
      // AM_DEBUG_INFO("error write!!!\r\n");
    }
    else{
      temp += 2;
      // AM_DEBUG_INFO("ok write!!!\r\n");
    }
  }

  WriteARF2498Reg(RFREG_PWR,0X00);
  WriteARF2498Reg(0x20,0x20); // 自动刷新信号强度
  SetDestinationAddress();	// 外设地址
  WriteARF2498Reg(0x0f,0x03);
  WriteARF2498Reg(0x10,0x04);
  WriteARF2498Reg(0x11,0x05);
  WriteARF2498Reg(0x12,0x06);
  WriteARF2498Reg(0x13,0x07);
  WriteARF2498Reg(0x14,0x08);
  WriteARF2498Reg(0x15,0x09);

  Flush_Tx_Fifo();
  Flush_Rx_Fifo();
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// 清除所有中断标志
  EnableRfInt();
  EnterRXMode();
  //
  set_power_callback(&sleep_funcs);
  init_aft2498 = 1;
}



/***********************************************************
发送无线数据
***********************************************************/
void SendRF(u8 *senddata,u8 sendnumber)
{
  u8 cnt;
  //GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
  ResetARF2498_CE_Port();
  Flush_Tx_Fifo();
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// 清除所有中断标志
  rf_data.mode = RF_SEND;
  Write_Fifo_Data(1,1,senddata, sendnumber);
  for(cnt =0;cnt<20;cnt++){
    if(ReadARF2498Reg(RFREG_State)&0x18){
      //AM_DEBUG_INFO("Write_Fifo_Data ok\r\n");
      break;
    }
    else{
      delay_ms(1);
      //DelayMs(1);
    }
  }
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// 清除所有中断标志
  delay_us(200);
  SetDestinationAddress();
  EnterRXMode();
}
/*******************************************************
	rf接收中的中断函数
*******************************************************/
//extern void get_schedule_data(u8 *data,u8 number);
void Set_virsual_ir_rx_code(void)
{
   u8 readdata[16];
   if(QueryARF2496KInceptSign()==TRUE)
   {
    ReadRF(readdata,16);
   }

   if((readdata[0]>= 0x1)&&(readdata[0]<4)&&(readdata[1] == 0xff))
   {
     //AM_DEBUG_INFO("readdata[1]=%x\r\n",readdata[1]);
       check_partnership_ok_flag = 1;
   }
   if(readdata[0]==0x01) //遥控器的信号
   {
   // AM_DEBUG_INFO("readdata[1]=%x\r\n",readdata[1]);
       ir_rx_code_get(1,readdata[1]);
       if(readdata[1] == 0x9)
       {
           //get_schedule_data(&readdata[2],6);
       }
   }
   else if(readdata[0]==0x02) //DOCK的信号
   {
    ir_rx_code_get(0,readdata[1]);
       dock_wireless_rx_code_get(0,readdata[1]);
       //AM_DEBUG_INFO("dock readdata[1]=%x\r\n",readdata[1]);
   }
   else if(readdata[0]==0x03)
   {
      //AM_DEBUG_INFO("readdata[0]=%x\r\n",readdata[0]);
      //AM_DEBUG_INFO("readdata[1]=%x\r\n",readdata[1]);
       ir_rx_code_get(0,readdata[1]);
   }
   signal_flag=1;
}

void RFIrq(void)
{
  u8 temp = 0;
  ResetARF2498_CE_Port();
  //GPIO_ResetBits(ARF2498_CE_Port, ARF2498_CE_Pin);	// 关闭接收
  temp = ReadARF2498Reg(RFREG_State);

  //if(temp&0x20){  // bit5 接收到一包数据
  if(temp&0x2){  // bit5 接收到一包数据
    //WriteARF2498Reg(RFREG_Rx_Chn,0);
    Read_Fifo_Data(rf_recvbuff);//spi
    //Rf_Send_Ack(pipe_numb-1, rf_askbuff, 5);
    RFInceptSign = TRUE;
    //GPIO_SetBits(ARF2498_CE_Port, ARF2498_CE_Pin);
    Set_virsual_ir_rx_code();
    SetARF2498_CE_Port();

  }
  WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// 清除所有中断标志
  Flush_Tx_Fifo();
  EnterRXMode();
}
/***********************************************************
从接收缓冲中接收数据,读出的数据不能大于16个字节
***********************************************************/
void ReadRF(u8 *readdata,u8 readnumber)
{
	for(;readnumber >0;readnumber--) //读出readnumber个数据到缓存中
	{
		readdata[readnumber-1] =  rf_recvbuff[readnumber-1];
	}
	RFInceptSign = FALSE;			  //读完之后清除标志
}
/***********************************************************
查询是否接收无线信号
***********************************************************/
bool QueryARF2496KInceptSign(void)
{
	return RFInceptSign;
}
/**********************************************************
	RF对码
**********************************************************/
void PartnershipRF(void)
{

	u16 i=0;
    //delay_us(200);
    SetPartnershipAddr();	//设置对码地址
//	gbv_rf_send_interval = 1000;

	rf_sendbuff[0] = HOST_ID;
	rf_sendbuff[1] = PARTNERSHIP;
	rf_sendbuff[2] = RFGroupAddress[0];
	rf_sendbuff[3] = RFGroupAddress[1];
	rf_sendbuff[4] = RFGroupAddress[2];
	rf_sendbuff[5] = RFGroupAddress[3];
	rf_sendbuff[6] = RFDeviceAddress;
	rf_sendbuff[15] = 0;
	for(;i<15;i++){
		rf_sendbuff[15] += rf_sendbuff[i];
	}
	//memset(rf_sendbuff,0x01,16);
	SendRF(rf_sendbuff,16);
   /*
	WriteARF2498Reg(RFREG_State,RF_IRQ_CLR);	// 清除所有中断标志
	delay_us(200);
	SetDestinationAddress();
	EnterRXMode();	*/

}

void Send_ACK_Package(void)
{
    u8 temp[16]={1};
    SendRF(temp,16);
}
void Wireless_Dock(void)
{
  u8 temp[2];
  // DisableRFIncept();
  // WriteDeviceAddressOfARF2496K();

  SetDestinationAddress();

  temp[0] = 0;
  temp[1] = 0x40;
  SendRF(temp,2);
 // DelayMs(2000);

 // EnableRFIncept();

}

void Wireless_Rcon(void)
{
  u8 temp[2];
  // DisableRFIncept();
  // WriteDeviceAddressOfARF2496K();

  SetDestinationAddress();

  temp[0] = 0;
  temp[1] = 0x20;
  SendRF(temp,2);
 // DelayMs(2000);

 // EnableRFIncept();

}
/****************************************************************
取得RF信号强度
****************************************************************/
u8 read_rf_Rssi(void)
{
	int i = 0;
	u16 sum = 0;
	for(;i<10;i++){
		sum += rssi_val[i];
	}
	return sum/10;
}

u8 get_partnership_ok_flag(void)
{
    return check_partnership_ok_flag;
}

void set_partnership_ok_flag(u8 partnership_ok_flag)
{
    check_partnership_ok_flag = partnership_ok_flag;
}

void opend_dock_signal(void)
{
  static u8 send_dock_command_cnd=4;
  static u32 time_flag = 0;
  clear_dock_wireless_rx_code();
  while(send_dock_command_cnd)
  {
    if(remote_ir_get((IR_REMOT_POSITION_E)0) ==0x40)
      break;
    else
    {
      if(timer_elapsed(time_flag)>10)
      {
        send_dock_command_cnd--;
        time_flag=timer_ms();
        Wireless_Dock() ;
      }
    }
  }
}

#endif




