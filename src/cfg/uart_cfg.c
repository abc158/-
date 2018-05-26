//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*by liyongyong*/
#include "am_uart_api.h"
#include "syscall_api.h"
#include "am_device.h"
#include "ui_uart_handle/mqueue.h"
#if defined(USE_WIFI_DEMO_1)
#include "wifi/wifi_demo1/simwifi.h"
#endif
#include  <am_config_macro.h>
//中断函数不调用库的远程函数，只能调用本地函数
int uart_debug;
int uart_ui;
int uart_wifi;
//uart 中断处理函数，sr是中断状态，data是接收的数据
//return int : 当为发送中断时，用作返回要发送的数据，-1，表示数据发完。
int uart_out_queue_is_empty()
{
  return 1;
}

int uart_out_queue_read()
{
  return 0;
}

int uart_out_queue_write(u16 val)
{
  return 0;
}

int uart_in_queue_write(u16 val)
{
  val = val;
  return 0;
}

int uart_in_queue_is_empty()
{
  return 1;
}

int uart_in_queue_read()
{
  return 0;
}


/****************************************************************
*Function   :  debug_uart_handler
*Author     :  lyy
*Date       :  2017.4.20
*Description:  打印口的中断处理函数
*CallBy     :  
*Input      :  参数
*              sr:    中断状态
               data： 接收到的数据
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
int debug_uart_handler(int sr, int data)
{
  int ret = -1;
  switch(sr)
  {
    case USART_IID_THRE://发送中断,这个只要打开发送中断,然后如果uart fifo为空，就会产生中，可以在这里处理要发送的数据
    	if(!uart_out_queue_is_empty())
        {
          ret = uart_out_queue_read();
        }
        else
        {
          ret  = -1;
        }
    	break; 
    case USART_IID_RDA :
    	uart_in_queue_write((uint8_t)data);
        ret = 0;
    	break;  
    case USART_IID_CTI :
    	break;   
    case USART_IID_MS  :
    	break;  
    case USART_IID_NON :
    	break;
    case USART_IID_RLS :  	
    	break;  
    default:
    	break;
  }
  return ret;
}


int wifi_uart_handler(int sr, int data)
{
  int ret = -1;
  if(sr == USART_IID_RDA)
  {   
    #if defined(USE_WIFI_DEMO_1)
      IRQ_Usart1RxData_Process((u8)(data&0xff));
    #elif defined(USE_WIFI_DEMO_2)
      if(uart_txrx_q_full(WIFI_RX_INDEX) == 0)
      {
          uart_txrx_q_put(WIFI_RX_INDEX,(u8)(data&0xff));
      }
    #endif    
  }
  
  if(sr == USART_IID_THRE)
  {
    if(uart_txrx_q_empty(WIFI_TX_INDEX))
    {
      ret = -1;
    }
    else
    {
      ret = uart_txrx_q_get(WIFI_TX_INDEX);//uart_out_3_getc( ) ; //
      //printf("ret %x\r\n",ret);
    }

    return ret;
  } 
  if(sr == USART_IID_CTI)
  {
  }
  if(sr == USART_IID_RLS)
  {
  }  
  return 0;
  //return ret;
}

void sys_uart_cfg(void)
{
  //UserDmaInit_t   UserDmaInit;
  UserUartInit_t  UserUartInit;
  uart_debug = open(DEV_UART,USART_ID1);  

  UserUartInit.UartInit.USART_BaudRate   = 115200;                 
  UserUartInit.UartInit.USART_WordLength = USART_WORDLENGTH_8B;  
  UserUartInit.UartInit.USART_StopBits   = USART_STOPBITS_1;       
  UserUartInit.UartInit.USART_Parity     = USART_PARITY_NO;     
  UserUartInit.UartInit.USART_Mode       = USART_MODE_NORMAL;          
  UserUartInit.UartInit.USART_LSB_MSB    = USART_LSB_FIRST;
	
  UserUartInit.UartMode.RxTxMode         = IRQ_MODE;//02:irq  0x1;//dma
  UserUartInit.UartMode.irqMode          = USART_IER_RDAIE;//0;//USART_IER_RDAIE;//no
  UserUartInit.UartMode.DmaMode          = DMA_RX_MODE;//1:rx  0:tx,rx 2:tx 
  UserUartInit.UartMode.RxTxEna          = UART_RXTX;//tx,rx
  
  ioctl(uart_debug,UART_INIT, &UserUartInit); 
  ioctl(uart_debug,UART_SET_CONSOLE, &uart_debug);//配置打印口
  request_irq(uart_debug, (long)debug_uart_handler,0); 
  


  #if defined(USE_UART_WIFI) 
  uart_wifi = open(DEV_UART,USART_ID0); 
  #if defined(USE_WIFI_DEMO_1)
  UserUartInit.UartInit.USART_BaudRate   = 38400;
  #elif defined(USE_WIFI_DEMO_2)
  UserUartInit.UartInit.USART_BaudRate   = 115200;
  #endif
  UserUartInit.UartInit.USART_WordLength = USART_WORDLENGTH_8B;  
  UserUartInit.UartInit.USART_StopBits   = USART_STOPBITS_1;       
  UserUartInit.UartInit.USART_Parity     = USART_PARITY_NO;//USART_PARITY_ODD;         
  UserUartInit.UartInit.USART_Mode       = USART_MODE_NORMAL;          
  UserUartInit.UartInit.USART_LSB_MSB    = USART_LSB_FIRST;
  UserUartInit.UartMode.RxTxMode         = IRQ_MODE;//CPU_MODE;//02:irq  0x1;//dma
  UserUartInit.UartMode.irqMode          = USART_IER_RDAIE;//0;//USART_IER_RDAIE;//no
  UserUartInit.UartMode.DmaMode          = 0;//DMA_RX_MODE;//1:rx  0:tx,rx 2:tx 
  UserUartInit.UartMode.RxTxEna          = UART_RXTX;//tx,rx
 // uart_init(wifi_uart_id, &UserUartInit);
  
  ioctl(uart_wifi,UART_INIT, &UserUartInit); 
  request_irq(uart_wifi, (long)wifi_uart_handler,0);
  #endif
}

void sys_uart_close(void)
{
  close(uart_debug);
 // close(uart_ui);
  close(uart_wifi);
}

void uart_test(void)
{
    static int cnt = 0;
    int i = 0;
    u8 trig = 1;
    //write(uart_debug,"123456",6); //直接发送
    while(!uart_in_queue_is_empty())
    {
      printf(" %x \r\n",uart_in_queue_read());
    }
    cnt++;
    if(cnt > 3)
    {
      for(i = 0;i < 32; i++)
      {
        uart_out_queue_write(i);//放入缓冲区，用中断发送
      }
      ioctl(uart_debug,UART_TRIG_TX,(void*)&trig);//触发一个发生中断
      cnt = 0;
    }  
}


void set_wifi_uart_tx_fifo_empty_irq(void)
{
    u8 trig = 1;
    ioctl(uart_wifi,UART_TRIG_TX,(void*)&trig);//触发一个发生中断
}
