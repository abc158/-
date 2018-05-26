#ifndef _GPTM_CFG_H_
#define _GPTM_CFG_H_

/**
 * @brief  Enumeration of GPTM counter mode.
 */
typedef enum
{
  GPTM_CNT_MODE_UP   = 0x00000000,  /*!< Edge up-counting mode */
  GPTM_CNT_MODE_CA1  = 0x00010000,  /*!< Center-align mode 1 */
  GPTM_CNT_MODE_CA2  = 0x00020000,  /*!< Center-align mode 2 */
  GPTM_CNT_MODE_CA3  = 0x00030000,  /*!< Center-align mode 3 */
  GPTM_CNT_MODE_DOWN = 0x01000000   /*!< Edge down-counting mode */
} GPTM_CNT_MODE_Enum;
/**
 * @brief Enumeration of  GPTM prescaler reload time.
 */
typedef enum
{
  GPTM_PSC_RLD_UPDATE    = 0x0000,  /*!< Reload prescaler at next update event */
  GPTM_PSC_RLD_IMMEDIATE = 0x0100   /*!< Reload prescaler immediately */
} GPTM_PSC_RLD_Enum;
/**
 * @brief Enumeration of GPTM channel.
 */
typedef enum
{
  GPTM_CH_0 = 0,                    /*!< GPTM channel 0 */
  GPTM_CH_1,                        /*!< GPTM channel 1 */
  GPTM_CH_2,                        /*!< GPTM channel 2 */
  GPTM_CH_3                         /*!< GPTM channel 3 */
} GPTM_CH_Enum;
/**
 * @brief Enumeration of GPTM channel control.
 */
typedef enum
{
  GPTM_CHCTL_DISABLE = 0,           /*!< GPTM channel disable */
  GPTM_CHCTL_ENABLE                 /*!< GPTM channel enable */
} GPTM_CHCTL_Enum;
/**
 * @brief Enumeration of GPTM channel polarity.
 */
typedef enum
{
  GPTM_CHP_NONINVERTED = 0,         /*!< GPTM channel polarity is active high or rising edge */
  GPTM_CHP_INVERTED                 /*!< GPTM channel polarity is active low or falling edge */
} GPTM_CHP_Enum;
/**
 * @brief Enumeration of GPTM channel capture source selection.
 */
typedef enum
{
  GPTM_CHCCS_DIRECT   = 0x00010000,  /*!< GPTM channel capture selection direct input  */
  GPTM_CHCCS_INDIRECT = 0x00020000,  /*!< GPTM channel capture selection indirect input  */
  GPTM_CHCCS_TRCED    = 0x00030000   /*!< GPTM channel capture selection TRCED of trigger control block */
} GPTM_CHCCS_Enum;
/**
 * @brief Enumeration of GPTM channel capture prescaler.
 */
typedef enum
{
  GPTM_CHPSC_OFF = 0x00000000,     /*!< GPTM channel capture no prescaler, capture is done each event */
  GPTM_CHPSC_2   = 0x00040000,     /*!< GPTM channel capture is done once every 2 event */
  GPTM_CHPSC_4   = 0x00080000,     /*!< GPTM channel capture is done once every 4 event */
  GPTM_CHPSC_8   = 0x000C0000      /*!< GPTM channel capture is done once every 8 event */
} GPTM_CHPSC_Enum;
/**
 * @brief Enumeration of GPTM fDTS clock divider.
 */
typedef enum
{
  GPTM_CKDIV_OFF = 0x0000,          /*!< fDTS = fCLKIN */
  GPTM_CKDIV_2   = 0x0100,          /*!< fDTS = fCLKIN / 2 */
  GPTM_CKDIV_4   = 0x0200           /*!< fDTS = fCLKIN / 4 */
} GPTM_CKDIV_Enum;
/**
 * @brief Enumeration of GPTM ETI input prescaler.
 */
typedef enum
{
  GPTM_ETIPSC_OFF = 0x00000000,     /*!< ETI prescaler off */
  GPTM_ETIPSC_2   = 0x00001000,     /*!< ETIP frequency divided by 2 */
  GPTM_ETIPSC_4   = 0x00002000,     /*!< ETIP frequency divided by 4 */
  GPTM_ETIPSC_8   = 0x00003000      /*!< ETIP frequency divided by 8 */
} GPTM_ETIPSC_Enum;
/**
 * @brief Enumeration of GPTM ETI input polarity.
 */
typedef enum
{
  GPTM_ETIPOL_NONINVERTED = 0x00000000, /*!< GPTM ETI polarity is active high or rising edge */
  GPTM_ETIPOL_INVERTED    = 0x00010000  /*!< GPTM ETI polarity is active low or falling edge */
} GPTM_ETIPOL_Enum;
/**
 * @brief Enumeration of GPTM slave trigger input selection.
 */
typedef enum
{
  GPTM_TRSEL_UEVG   = 0x0,         /*!< Software trigger by seting UEVG bit */
  GPTM_TRSEL_TAM38xS0  = 0x1,         /*!< Filtered channel 0 input */
  GPTM_TRSEL_TI1S1  = 0x2,         /*!< Filtered channel 1 input */
  GPTM_TRSEL_ETIF   = 0x3,         /*!< External Trigger input */
  GPTM_TRSEL_TAM38xBED = 0x8,         /*!< Trigger input 0 both edge detector */
  GPTM_TRSEL_ITAM38x   = 0x9,         /*!< Internal trigger 0 */
  GPTM_TRSEL_ITI1   = 0xA,         /*!< Internal trigger 1 */
  GPTM_TRSEL_ITI2   = 0xB          /*!< Internal trigger 2 */
} GPTM_TRSEL_Enum;
/**
 * @brief Enumeration of GPTM slave mode selection.
 */
typedef enum
{
  GPTM_SMSEL_DISABLE  = 0x0000,     /*!< The prescaler is clocked directly by the internal clock */
  GPTM_SMSEL_DECODER1 = 0x0100,     /*!< Counter counts up/down on CH0 edge depending on CH1 level */
  GPTM_SMSEL_DECODER2 = 0x0200,     /*!< Counter counts up/down on CH1 edge depending on CH0 level */
  GPTM_SMSEL_DECODER3 = 0x0300,     /*!< Counter counts up/down on both CH0 & CH1 edges depending on the
                                       level of the other input */
  GPTM_SMSEL_RESTART  = 0x0400,     /*!< Slave restart mode */
  GPTM_SMSEL_PAUSE    = 0x0500,     /*!< Slave pause mode */
  GPTM_SMSEL_TRIGGER  = 0x0600,     /*!< Slave trigger mode */
  GPTM_SMSEL_STIED    = 0x0700      /*!< Rising edge of the selected trigger(STI) clock the counter */
} GPTM_SMSEL_Enum;
/**
 * @brief  Enumeration of GPTM master mode selection.
 */
typedef enum
{
  GPTM_MMSEL_RESET   = 0x00000000,  /*!< Send trigger signal when S/W setting UEVG or slave restart */
  GPTM_MMSEL_ENABLE  = 0x00010000,  /*!< The counter enable signal is used as trigger output. */
  GPTM_MMSEL_UPDATE  = 0x00020000,  /*!< The update event is used as trigger output. */
  GPTM_MMSEL_CH0CC   = 0x00030000,  /*!< Channel 0 capture or compare match occurred as trigger output. */
  GPTM_MMSEL_CH0OREF = 0x00040000,  /*!< The CH0OREF signal is used as trigger output. */
  GPTM_MMSEL_CH1OREF = 0x00050000,  /*!< The CH1OREF signal is used as trigger output. */
  GPTM_MMSEL_CH2OREF = 0x00060000,  /*!< The CH2OREF signal is used as trigger output. */
  GPTM_MMSEL_CH3OREF = 0x00070000   /*!< The CH3OREF signal is used as trigger output. */
} GPTM_MMSEL_Enum;
/**
 * @brief Enumeration of GPTM channel Capture / Compare PDMA selection.
 */
typedef enum
{
  GPTM_CHCCDS_CHCCEV = 0,       /*!< Send CHx PDMA request when channel capture/compare event occurs */
  GPTM_CHCCDS_UEV               /*!< Send CHx PDMA request when update event occurs */
} GPTM_CHCCDS_Enum;
/* Exported constants --------------------------------------------------------------------------------------*/
/*GPTM_Exported_Constants GPTM exported constants */

/* Definitions of GPTM_INT */
#define GPTM_INT_CH0CC        0x0001  /*!< Channel 0 capture/compare interrupt */
#define GPTM_INT_CH1CC        0x0002  /*!< Channel 1 capture/compare interrupt */
#define GPTM_INT_CH2CC        0x0004  /*!< Channel 2 capture/compare interrupt */
#define GPTM_INT_CH3CC        0x0008  /*!< Channel 3 capture/compare interrupt */
#define GPTM_INT_UEV          0x0100  /*!< Update interrupt */
#define GPTM_INT_TEV          0x0400  /*!< Trigger interrupt */

/* GPTM channel output modes */
#define GPTM_OM_MATCH_NOCHANGE        0x0000  /*!< GPTM channel output no change on match */
#define GPTM_OM_MATCH_INACTIVE        0x0001  /*!< GPTM channel output inactive level on match */
#define GPTM_OM_MATCH_ACTIVE          0x0002  /*!< GPTM channel output active level on match */
#define GPTM_OM_MATCH_TOGGLE          0x0003  /*!< GPTM channel output toggle on match */
#define GPTM_OM_FORCED_INACTIVE       0x0004  /*!< GPTM channel output forced inactive level */
#define GPTM_OM_FORCED_ACTIVE         0x0005  /*!< GPTM channel output forced active level */
#define GPTM_OM_PWM1                  0x0006  /*!< GPTM channel pwm1 output mode */
#define GPTM_OM_PWM2                  0x0007  /*!< GPTM channel pwm2 output mode */
#define GPTM_OM_ASYMMETRIC_PWM1       0x0106  /*!< GPTM channel asymmetric pwm1 output mode */
#define GPTM_OM_ASYMMETRIC_PWM2       0x0107  /*!< GPTM channel asymmetric pwm2 output mode */

/* Definitions of GPTM_PDMA */
#define GPTM_PDMA_CH0CC        0x00010000  /*!< Channel 0 capture/compare PDMA request */
#define GPTM_PDMA_CH1CC        0x00020000  /*!< Channel 1 capture/compare PDMA request */
#define GPTM_PDMA_CH2CC        0x00040000  /*!< Channel 2 capture/compare PDMA request */
#define GPTM_PDMA_CH3CC        0x00080000  /*!< Channel 3 capture/compare PDMA request */
#define GPTM_PDMA_UEV          0x01000000  /*!< Update PDMA request */
#define GPTM_PDMA_TEV          0x04000000  /*!< Trigger PDMA request */

/* Definitions of GPTM_EVENT */
#define GPTM_EVENT_CH0CC      0x0001  /*!< Channel 0 capture/compare event */
#define GPTM_EVENT_CH1CC      0x0002  /*!< Channel 1 capture/compare event */
#define GPTM_EVENT_CH2CC      0x0004  /*!< Channel 2 capture/compare event */
#define GPTM_EVENT_CH3CC      0x0008  /*!< Channel 3 capture/compare event */
#define GPTM_EVENT_UEV        0x0100  /*!< Update event */
#define GPTM_EVENT_TEV        0x0400  /*!< Trigger event */

/* Definitions of GPTM_FLAG */
#define GPTM_FLAG_CH0CC       0x0001  /*!< Channel 0 capture/compare flag */
#define GPTM_FLAG_CH1CC       0x0002  /*!< Channel 1 capture/compare flag */
#define GPTM_FLAG_CH2CC       0x0004  /*!< Channel 2 capture/compare flag */
#define GPTM_FLAG_CH3CC       0x0008  /*!< Channel 3 capture/compare flag */
#define GPTM_FLAG_CH0OC       0x0010  /*!< Channel 0 over capture flag */
#define GPTM_FLAG_CH1OC       0x0020  /*!< Channel 1 over capture flag */
#define GPTM_FLAG_CH2OC       0x0040  /*!< Channel 2 over capture flag */
#define GPTM_FLAG_CH3OC       0x0080  /*!< Channel 3 over capture flag */
#define GPTM_FLAG_UEV         0x0100  /*!< Update flag */
#define GPTM_FLAG_TEV         0x0400  /*!< Trigger flag */


#endif