/* mqueue.h - interface to Mobject queue implementations */


#ifndef MQUEUE_H
#define MQUEUE_H

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include "am_type.h"

/**********************************/
/*** Queue - Fixed Message Size ***/
/**********************************/

/* queue configuration and state structures, JWM generate from template */

typedef struct
{
  uint16_t length;
  uint16_t msgSize;
} queue_cfg_t;

typedef struct
{
  uint16_t nItems;
  uint16_t readIndex;
  uint8_t  bufStart[1];
} queue_st_t;

typedef struct
{
  queue_cfg_t cfg;
  queue_st_t  *st_p;
} queue_info_t;

typedef struct
{
  uint8_t  length;
  uint8_t  msgSize;
} queuec_cfg_t;

typedef struct
{
  uint8_t  nItems;
  uint8_t  readIndex;
  uint8_t  bufStart[1];
} queuec_st_t;

typedef struct
{
  queuec_cfg_t cfg;
  queuec_st_t  *st_p;
} queuec_info_t;


/* API Functions */


extern uint16_t data_length2;
extern void uart_txrx_init(void);
extern void uart_txrx_q_put(uint8_t index, uint8_t value);
extern uint8_t uart_txrx_q_get(uint8_t index);
extern uint8_t uart_txrx_q_empty(uint8_t index);
extern uint8_t uart_txrx_q_full(uint8_t index);
extern uint16_t uart_txrx_get_datalength(uint8_t index);
extern uint16_t uart_txrx_get_space(uint8_t index);
extern void uart_txrx_reset_writeptr(uint8_t index);
extern void uart_txrx_reset_readptr(uint8_t index);
extern void uart_txrx_reset_datalength(uint8_t index);
extern uint8_t uart_txrx_q_get_offset(uint8_t index,uint8_t offset, uint8_t len, uint8_t *buf);
/* macros needed by API Macros */



#define RX_INDEX 0
#define TX_INDEX 1  
#define WIFI_RX_INDEX 2
#define WIFI_TX_INDEX 3 
#endif  /* MQUEUE_H */


