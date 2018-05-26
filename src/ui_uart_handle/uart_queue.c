/* uiqueue.c - uart tx rx 队列管理 */

#include "mqueue.h"
#include "am_type.h"
#include "syscall_api.h"
#include "am_config_macro.h"
#define QUEUE_LEN  100
#define QUEUE_LEN2  100

#if defined(USE_UART_WIFI)
/* function templates */
U8 txrx_queue[2][QUEUE_LEN];
U8 wifi_rx_queue[2][QUEUE_LEN2];
U16 write_ptr[4];
U16 read_ptr[4];
U16 data_length[4];

U16 data_length2 = 0;

U16 queue_len[4] = {QUEUE_LEN,QUEUE_LEN,QUEUE_LEN2,QUEUE_LEN2};


void uart_txrx_init(void)
{
    memset(txrx_queue, 0, sizeof(txrx_queue));
    memset(write_ptr, 0, sizeof(write_ptr));
    memset(read_ptr, 0, sizeof(read_ptr));
    memset(data_length, 0, sizeof(data_length));
    memset(wifi_rx_queue,0,sizeof(wifi_rx_queue));
    //memset(working_byte, 0, sizeof(working_byte));
    //memset(capture_counter, 0, sizeof(capture_counter));
}

void uart_txrx_q_put(U8 index, U8 value)
{
	 u8 tmp;
	 tmp = write_ptr[index];

    if(data_length[index] < queue_len[index])
    {
        if(index < 2)
        {
            txrx_queue[index][tmp] = value;
        }
        else
        {
            wifi_rx_queue[index-2][tmp] = value;
        }
        
        write_ptr[index]++;
        if (write_ptr[index] >= queue_len[index])
        {
            write_ptr[index] = 0;
        }
        data_length[index]++;
    }
		
    #if 0
    if(index == 0)
    {
        data_length2++;
        if(data_length2 > 65000)
        {
            data_length2 = 0;
        }
    }
    #endif
}

U8 uart_txrx_q_get(U8 index)
{
    U8 state;
	u8 tmp; 
	tmp = read_ptr[index];

	if(data_length[index] > 0)
	{
        if(index < 2)
        {
            state = txrx_queue[index][tmp];         
        }
        else
        {
            state = wifi_rx_queue[index-2][tmp]; 
        }
        read_ptr[index]++;
        if (read_ptr[index] >= queue_len[index])
        {
            read_ptr[index] = 0;
        }
        data_length[index]--;
	}
    //hal_ints_on( );
    return state;  
  
}

U8 uart_txrx_q_empty(U8 index)
{
    if (data_length[index] == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

U8 uart_txrx_q_get_offset(U8 index,U8 offset, U8 len, U8 *buf)
{
  int readptr = read_ptr[index]+offset;
  int i = 0;
  U16 state;
  if( data_length[index] >= (offset+len) )
  {
    while(i<len)
    {
      if(index < 2)
      {
         state = txrx_queue[index][readptr];
      }
      else
      {
         state = wifi_rx_queue[index-2][readptr]; 
      }
      readptr++;
      if (readptr >= queue_len[index])
      {
        readptr = 0;
      }
      buf[i] = (U8)state;
      i++;
    }
  }  
  return 0;
}

U8 uart_txrx_q_full(U8 index)
{
    if (data_length[index] >= queue_len[index])
    {
        if(index == WIFI_RX_INDEX)
        {
            printf("q_full!!\r\n");
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

U16 uart_txrx_get_datalength(U8 index)
{
    return data_length[index];
}

U16 uart_txrx_get_space(U8 index)
{
    if(data_length[index] > queue_len[index])
    {
        return 0;
    }
    else
    {
        return (queue_len[index] - data_length[index]);
    }
}
void uart_txrx_reset_writeptr(U8 index)
{
    write_ptr[index] = 0;
}

void uart_txrx_reset_readptr(U8 index)
{
    read_ptr[index] = 0;
}

void uart_txrx_reset_datalength(U8 index)
{
    data_length[index] = 0;
}
#endif
