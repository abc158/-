//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif 

#ifndef AMUIPC_H
#define AMUIPC_H


#include "mqueue.h"


#if !defined(AMUIPC_RX_INACTIVITY_TICKS)
#define AMUIPC_RX_INACTIVITY_TICKS    3 
#endif



#define AMUIPC_PLI_SIZE 2
#define AMUIPC_END_FLAG 0x55
#define AMUIPC_BOARD_INFO 0x05
typedef enum
{
  AMUIPC_PAYLOAD_ID_NONE      = 0,
  AMUIPC_PAYLOAD_ID_LM        = 1,  
                                     
  AMUIPC_PAYLOAD_ID_TL        = 7,  
  AMUIPC_PAYLOAD_ID_AMPC       = 8,  
  AMUIPC_PAYLOAD_ID_STP       = 9, 
  AMUIPC_PAYLOAD_ID_STPA      = 10,  
  AMUIPC_PAYLOAD_ID_USER      = 16  
} AMUIPC_PAYLOAD_ID_E;


typedef enum
{
  AMUIPC_RET_OK = 0,
  AMUIPC_Q_FULL
} AMUIPC_RET_E;

typedef enum
{
  AMUIPC_UIPC_ID_NULL            = 0,
  AMUIPC_UIPC_ID_BL_INFO         = 16,
  AMUIPC_UIPC_ID_APP_IMAGE_STATUS= 17,
  AMUIPC_UIPC_ID_CMD_CTR         = 20,
  AMUIPC_UIPC_ID_CMD_GET         = 21,
  AMUIPC_UIPC_ID_DATA            = 22 
} AMUIPC_UIPC_OBJECT_ID_E;





#define AMUIPC_FC_OFFSET             0
#define AMUIPC_OBJ_ID_OFFSET         1
#define AMUIPC_OBJ_VALUE_OFFSET      2


typedef enum
{
  AMUIPC_FC_MASK               = 0x3F,
  AMUIPC_FC_NONE               = 0x00,
  AMUIPC_FC_OBJ_COMMAND        = 0x02,
  AMUIPC_FC_OBJ_CONTROL        = 0x04,
  AMUIPC_FC_OBJ_STATUS         = 0x05, 
  AMUIPC_FC_OPT_MASK           = 0xC0,
  AMUIPC_FC_RESP_MASK          = 0x40,
  AMUIPC_FC_OPT_NO_RESP        = 0x00,
  AMUIPC_FC_OPT_RESP_OK        = 0x40,
  AMUIPC_FC_OPT_REQ_RESP       = 0x80,
  AMUIPC_FC_OPT_RESP_EXCEPTION = 0xC0
} AMUIPC_FUNC_CODE_E;


typedef enum
{
  AMUIPC_EXCEPTION_NONE            = 0x00,  
  AMUIPC_EXCEPTION_FC_INVALID      = 0x01,  
  AMUIPC_EXCEPTION_FC_INACTIVE     = 0x02,  
  AMUIPC_EXCEPTION_PARAM_ERR       = 0x03, 
  AMUIPC_EXCEPTION_DEVICE_BUSY     = 0x04,  
  AMUIPC_EXCEPTION_USER_DEFINED    = 0x80

} AMUIPC_EXCEPTION_CODE_E;



typedef struct
{
  U8  payloadId;
  U8  len;
  U8  *pkt_p;
} amuipc_pkt_info_t;


extern U8 crc8_buffer(U8 len , U8 *buf_p , U8 initVal );
extern U16 crc16_buffer( U8 len , U8 *buf_p , U16 initVal );
extern U8 igfp_tx_crc8_l2_pkt( 
                               U8   payloadId,
                               U8   len,
                               U8   *pkt_p );
#endif

#ifdef __cplusplus
}
#endif
