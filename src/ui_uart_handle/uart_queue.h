/* uiqueue.c - uart tx rx 队列管理 */
#ifndef _UIQUEUE_H_
#define _UIQUEUE_H_

#include "mqueue.h"
#include "am_type.h"

#define QUEUE_LEN  256

void uart_txrx_init(void);
void uart_txrx_q_put(U8 index, U8 value);
U8 uart_txrx_q_get(U8 index);
U8 uart_txrx_q_empty(U8 index);
U8 uart_txrx_q_full(U8 index);
U16 uart_txrx_get_datalength(U8 index);
U16 uart_txrx_get_space(U8 index);
void uart_txrx_reset_writeptr(U8 index);
void uart_txrx_reset_readptr(U8 index);
void uart_txrx_reset_datalength(U8 index);

#endif