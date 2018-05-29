#ifndef ___IR_H__
#define ___IR_H__

#include "am_type.h"
#include "syscall_api.h"
#include "am_gpio.h"
#define IR_QUEUE_LEN 128

#define REMOTE_DATA_KEEP_COUNT  20

#define IR_DECODER_MAX 6

typedef enum
{
  REMOTE_RESYNC,
  REMOTE_BYTE_START_WAIT,
  REMOTE_START_BIT,
  REMOTE_DATA_BIT,
  REMOTE_STOP_BIT,
  REMOTE_START_BIT_WAIT,
  REMOTE_STOP_BIT_WAIT,
  REMOTE_BYTE_STOP_WAIT,
} REMOTE_DECODE_ST_E;


typedef struct
{
  REMOTE_DECODE_ST_E state;
  unsigned char    timer;
  unsigned char    count;
  unsigned char    bitsDecoded;
  unsigned char	   rxByte;
} REMOTE_DECODE;


#define REMOTE_DECODER_MAX 6


typedef enum{
  IR_POSITION_MID_LEFT,
  IR_POSITION_MID_RIGHT,
  IR_POSITION_LEFT,
  IR_POSITION_RIGHT,
  IR_POSITION_BACK_LEFT,
  IR_POSITION_BACK_RIGHT,
}IR_REMOT_POSITION_E;

#define E_SLAM

typedef enum
{
  IR_CODE_POWER      = 0xC1,
  IR_CODE_DOCK       = 0xC2,
  IR_CODE_START_STOP       = 0xC3,
  IR_CODE_FORWARD    = 0xC4,
  IR_CODE_LEFT       = 0xC5,
  IR_CODE_RIGHT      = 0xC6,
  IR_CODE_BACKWARD   = 0xC7,
  IR_CODE_LIGHT      = 0xC8,
  IR_CODE_MUSIC      = 0xC9,
  IR_CODE_SPOT       = 0xCA,
  IR_CODE_SCHEDULE   = 0xCB,
  IR_CODE_CLOCK  = 0xCC,

  IR_CODE_START  =0xff,/* 以下与遥控不符合，后续去掉*/
  IR_CODE_DRY   = 0xc6,
  IR_CODE_DAMP  = 0xc4,
  IR_CODE_CLEAN_MODE = 0x8a,
  IR_CODE_SWAP = 0xff,
  IR_CODE_STOP = 0xC7,
  IR_CODE_PAUSE= 0xc5,
} IR_KEY_CODE;


void handle_ir_remote(void);
void ir_rx_capture_4khz(void);
uint8_t remote_ir_get(IR_REMOT_POSITION_E index);

#endif  /* ___IR_H__ */

