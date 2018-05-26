//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "ui-ir.h"

#define REMOTE_RESYNC_TICKS 14
#define REMOTE_START_BIT_TICK 3
#define REMOTE_DATA_BIT_TICK 7
#define REMOTE_STOP_BIT_TICK 14
#define REMOTE_BIT_TIMEOUT_TICK 18
#define REMOTE_BYTE_STOP_TICKS 8
#define REMOTE_STOP_BIT_MIN_TICKS 2


REMOTE_DECODE remote_decode[REMOTE_DECODER_MAX];

void remote_resync_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_RESYNC;
  decode_p->timer = 0;
}

void remote_byte_start_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_BYTE_START_WAIT;
}

void remote_start_bit_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_START_BIT;
  decode_p->timer = 0;
}

void remote_data_bit_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_DATA_BIT;
  decode_p->count = 0;
}

void remote_stop_bit_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_STOP_BIT_WAIT;
}

void remote_stop_bit_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_STOP_BIT;
  decode_p->count = 0;
}

void remote_start_bit_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_START_BIT_WAIT;
}

void remote_byte_stop_wait_init( REMOTE_DECODE *decode_p )
{
  decode_p->state = REMOTE_BYTE_STOP_WAIT;
  decode_p->timer = 0;
}

/****************************************************************
*Function   :  remote_decode_init
*Description:  获取接收头接收到遥控器的数据 
*Input      :  u16 instance:接收头ID            
*Output     :  无
*Return     :  -1,出错 0正确
*Others     :  
******************************************************************/
u16 remote_decode_init( u16 instance )
{
 	
  REMOTE_DECODE *decode_p;

  if (instance >= REMOTE_DECODER_MAX)
  {
    return(-1);
  }
  decode_p = &remote_decode[instance];

  remote_resync_init(decode_p);

  return(0);
   
}

/****************************************************************
*Function   :  remote_decode_ir
*Description:  获取接收头接收到遥控器的数据 
*Input      :  u16 instance:接收头ID   
*              u16 ir_state:接收头采集到的电平
*Output     :  无
*Return     :  -1:未解出正确的编码 0:解出正确的编码
*Others     :  此处红外的编码方式是"1"为高电平持续3ms,低电平持续1ms
                                   "0"为高电平持续1ms,低电平持续3ms
******************************************************************/
int16_t remote_decode_ir( u16 instance,
                           u16 ir_state )
{
 
  REMOTE_DECODE *decode_p;
  int16_t     remoteStatus = -1;
  

  if (instance >= REMOTE_DECODER_MAX)
  {
    return(-1);
  }
  decode_p = &remote_decode[instance];


  decode_p->timer++;

  switch (decode_p->state)
  {
  case REMOTE_RESYNC:
    if (ir_state)
	  { 
	    remote_resync_init(decode_p);
	  }
	  else if (decode_p->timer >= REMOTE_RESYNC_TICKS)
	  { 
	    remote_byte_start_wait_init(decode_p);
	  }
    break;
    
  case REMOTE_BYTE_START_WAIT:
    if (ir_state)
	  {
	    decode_p->bitsDecoded = 0;
	    remote_start_bit_init(decode_p);
	  }
    break;
      
  case REMOTE_START_BIT:
    if (decode_p->timer >= REMOTE_START_BIT_TICK)
	  {
		  remote_data_bit_init(decode_p);
	  }
    break;
    
  case REMOTE_DATA_BIT:
    if (ir_state)
	  {
      decode_p->count++;
	  }
    if (decode_p->timer >= REMOTE_DATA_BIT_TICK)
	  { 
		  decode_p->rxByte <<= 1;
		  switch (decode_p->count)
		  {
		  case 0:
		  case 1:
		    remote_stop_bit_wait_init(decode_p);
		    break;
		  case 2:
		    remote_resync_init(decode_p);
		    break;
		  default:
		    decode_p->rxByte++;
		    remote_stop_bit_wait_init(decode_p);
		    break;
		  }
	  }
    break;
  
  case REMOTE_STOP_BIT_WAIT:
    remote_stop_bit_init(decode_p);
    break;

  case REMOTE_STOP_BIT:
    if (ir_state == 0)
	  {
      decode_p->count++;
      if (decode_p->count >= REMOTE_STOP_BIT_MIN_TICKS)
      { 
	      decode_p->bitsDecoded++;
	      if (decode_p->bitsDecoded >= 8)
	      {
	        remote_byte_stop_wait_init(decode_p);
	      }
	      else
	      { 
	        remote_start_bit_wait_init(decode_p);
	      }
      }
	  }
	  else
    {
      decode_p->count = 0;
      if (decode_p->timer >= REMOTE_STOP_BIT_TICK)
	    {
	      remote_resync_init(decode_p);
	    }
    }
    break;

  case REMOTE_START_BIT_WAIT:
    if (ir_state)
	  { 
	    remote_start_bit_init(decode_p);
	  }
	  else if (decode_p->timer >= REMOTE_BIT_TIMEOUT_TICK)
	  { 
	    remote_resync_init(decode_p);
	  }
	  break;

  case REMOTE_BYTE_STOP_WAIT:
    if (ir_state)
    { 
      remote_resync_init(decode_p);
    }
    else if (decode_p->timer >= REMOTE_BYTE_STOP_TICKS)
    { 
      remote_byte_start_wait_init(decode_p);
      remoteStatus = 0;
    }
    break;

  default:
    break;
  }
  
  return(remoteStatus);
   
}
  



