#ifndef _TM_API_H_
#define _TM_API_H_
#include <stdint.h>
#include <am_gptm_cfg.h>
#include "AM38x_mctm.h"

#define SystemCoreClock 36000000
#define FREQ_4KHZ  (SystemCoreClock/4000)-1 //
#define FREQ_8KHZ  (SystemCoreClock/8000)-1 //
#define FREQ_64HZ  (SystemCoreClock/64)-1 //

typedef void (*fun_ptr)(int);

typedef struct
{
   fun_ptr fun_handle;
}irq_handler_t;


#define MAIN_BRUSH_PWM_CHANNEL    3
#define SIDE_BRUSH_PWM_CHANNEL    2
#define VACUUM_PWM_CHANNEL        4
#define BIN_LEFT_PWM_CHANNEL      7
#define BIN_RIGHT_PWM_CHANNEL     8
#define CHARGER_PWM_CHANNEL       9
#define RIGHT_WHEEL_PWM_CHANNEL   10
#define LEFT_WHEEL_PWM_CHANNEL    11
#define HOU1_IR_SEND_PWM_CHANNEL       12
#define HOU2_IR_SEND_PWM_CHANNEL       13
#define QIAN1_IR_SEND_PWM_CHANNEL       14
#define QIAN2_IR_SEND_PWM_CHANNEL       15

//////////////////基本定时器///////////////////
typedef enum
{
  BF_TM1_ID = 1,
  BF_TM2_ID = 2,
  BF_TM3_ID = 3
}BF_TM_E;

typedef enum
{
  BFTM_MIME           = 0x1,
  BFTM_ONE_SHOT       = 0x2
} BFTM_CNT_MODE_Enum;

typedef struct
{
  uint32_t           counter;
  uint32_t           compare;
  BFTM_CNT_MODE_Enum Mode;
} BFTM_TimeBaseInitTypeDef;
/////////////////////////////////////////////

////////////////通用定时器///////////////////
typedef enum
{
  GP_TM_NULL =0,
  GP_TM0_ID = 1,
  GP_TM1_ID = 2,
  GP_TM2_ID = 3,
}GP_TM_E;
/**
 * @brief Definition of GPTM timebase init structure.
 */
typedef struct
{
  uint16_t           CounterReload; /*!< Period (Value for CRR register) */
  uint16_t           Prescaler;     /*!< Prescaler (Value for PSCR register) */
  GPTM_CNT_MODE_Enum CounterMode;   /*!< Counter mode refer to \ref GPTM_CNT_MODE_Enum */
  GPTM_PSC_RLD_Enum  PSCReloadTime; /*!< Prescaler reload mode refer to \ref GPTM_PSC_RLD_Enum */
} GPTM_TimeBaseInitTypeDef_t;
/**
 * @brief Definition of GPTM channel output init structure.
 */
typedef struct
{
  GPTM_CH_Enum    Channel;              /*!< Channel selection refer to \ref GPTM_CH_Enum */
  uint32_t        OutputMode;           /*!< Channel output mode selection refer to \ref GPTM_OM_Enum */
  GPTM_CHCTL_Enum Control;              /*!< CHxO output state refer to \ref GPTM_CHCTL_Enum */
  GPTM_CHP_Enum   Polarity;             /*!< CHxO polarity refer to \ref GPTM_CHP_Enum */
  uint16_t        Compare;              /*!< Value for CHxCCR register */
  uint16_t        AsymmetricCompare;    /*!< Value for CHxCCR register */
} GPTM_OutputInitTypeDef_t;
/**
 * @brief Definition of GPTM channel input init structure.
 */
typedef struct
{
  GPTM_CH_Enum    Channel;         /*!< Channel selection refer to \ref GPTM_CH_Enum */
  GPTM_CHP_Enum   Polarity;        /*!< Channel input polarity refer to \ref GPTM_CHP_Enum */
  GPTM_CHCCS_Enum Selection;       /*!< Channel capture source selection refer to \ref GPTM_CHCCS_Enum */
  GPTM_CHPSC_Enum Prescaler;       /*!< Channel Capture prescaler refer to \ref GPTM_CHPSC_Enum */
  uint8_t         Filter;          /*!< Digital filter Confuration, it must between 0x0 ~ 0xF. */
} GPTM_CaptureInitTypeDef_t;

typedef struct
{
  uint16_t   event;      /* Definitions of GPTM_EVENT */
  uint16_t   irq;        /* Definitions of GPTM_INT */
  uint32_t   dmaReqMode; /*Definitions of GPTM_PDMA */
} GPTM_OutModeTypeDef;

typedef struct
{
  uint8_t         uevg_en; //1: en,  0: not en
  GPTM_TRSEL_Enum tri_sel;   //
  GPTM_SMSEL_Enum slave_mode;//
  u32        irq_mode;  /* Definitions of GPTM_INT */
}GPTM_CapModeTypeDef;

////////输入capture模式
typedef struct
{
  GPTM_CaptureInitTypeDef_t CapInit;
  GPTM_CapModeTypeDef     CapMode;
}UserCapInit_t;
////////输出pwm模式配置
typedef struct
{
  GPTM_OutputInitTypeDef_t OutInit;
  GPTM_OutModeTypeDef    OutMode;
}UserOutInit_t;
////////////////////////////////////////////////////

typedef struct
{
  uint16_t   irq;  /* Definitions of GPTM_INT */
  uint32_t   dmaReqMode; /*Definitions of GPTM_PDMA */
} MCTM_OutModeTypeDef;

typedef struct
{
	EventStatus     uevg_en;
  MCTM_TRSEL_Enum tri_sel;   //
  MCTM_SMSEL_Enum slave_mode;//
  uint32_t        irq_mode;  /* Definitions of GPTM_INT */
}MCTM_CapModeTypeDef;

typedef struct
{
  MCTM_CaptureInitTypeDef CapInit;
  MCTM_CapModeTypeDef     CapMode;
}MUserCapInit_t;
//////////////////////////////////////
typedef struct
{
  MCTM_OutputInitTypeDef OutInit;
  MCTM_OutModeTypeDef    OutMode;
}MUserOutInit_t;
////////////////////////////////////
typedef enum
{
  MC_TM_NULL=0,
  MC_TM1_ID = 0x1,
  MC_TM2_ID = 0x2,
}MC_TM_E;

void sys_tm_cfg(void);
void sys_tm_close(void);
void sys_set_pwm_out_duty(s16 channel , s16 Dutyfactor);
#endif