#ifndef _GPIO_H_
#define _GPIO_H_
#include <stdint.h>
#include <am_type.h>

#define GPIO_F_DIR_OUT (0<<0)
#define GPIO_F_DIR_IN  (1<<0)

#define GPIO_F_INIT_LOW  (0<<1)
#define GPIO_F_INIT_HIGH  (1<<1)

#define GPIO_F_OUT_INIT_LOW  (GPIO_F_DIR_OUT | GPIO_F_INIT_LOW)
#define GPIO_F_OUT_INIT_HIGH  (GPIO_F_DIR_OUT | GPIO_F_INIT_HIGH)

#define GPIO_F_OPEN_DRAIN  (1<<2)
#define GPIO_F_OPEN_SOURCE  (1<<3)

#define PA_NR 16
#define PB_NR 16
#define PC_NR 16
#define PD_NR 16
#define PE_NR 16
#define PF_NR 16
#define PG_NR 6

#define AW_GPIO_NEXT(gpio) gpio##_NR_BASE + gpio##_NR

enum gpio_number {
	PA_NR_BASE = 0,
	PB_NR_BASE = AW_GPIO_NEXT(PA),
	PC_NR_BASE = AW_GPIO_NEXT(PB),
	PD_NR_BASE = AW_GPIO_NEXT(PC),
	PE_NR_BASE = AW_GPIO_NEXT(PD),
	PF_NR_BASE = AW_GPIO_NEXT(PE),
	PG_NR_BASE = AW_GPIO_NEXT(PF),			
};

#define GPIOA(n) (n + PA_NR_BASE)
#define GPIOB(n) (n + PB_NR_BASE)
#define GPIOC(n) (n + PC_NR_BASE)
#define GPIOD(n) (n + PD_NR_BASE)
#define GPIOE(n) (n + PE_NR_BASE)
#define GPIOF(n) (n + PF_NR_BASE)
#define GPIOG(n) (n + PG_NR_BASE)

#define ARCH_NR_GPIOS (102)

typedef enum
{
  GPIO_PR_UP = 0,   /*!< weak pull-up resistor */
  GPIO_PR_DOWN,     /*!< weak pull-down resistor */
  GPIO_PR_DISABLE   /*!< Tri-state */
}GPIO_PR_Enum ;

/**
 * @brief  Enumeration of GPIO output drive current.
 */
typedef enum
{
	GPIO_DV_H_6MA = 0,   /*!< 4mA source/sink current */
	GPIO_DV_L_6MA,
	GPIO_DV_H_3MA,        /*!< 8mA source/sink current */
	GPIO_DV_L_3MA
}GPIO_DV_Enum;
/**
 * @brief  Enumeration of GPIO direction.
 */
typedef enum
{
  GPIO_DIR_IN = 0,    /*!< input mode */
  GPIO_DIR_OUT        /*!< output mode */
}GPIO_DIR_Enum ;

typedef enum
{
  PR_UP = 0,   /*!< weak pull-up resistor */
  PR_DOWN,     /*!< weak pull-down resistor */
  PR_DISABLE   /*!< Tri-state */
}GPIO_PR_E ;


typedef enum
{
  DIR_IN = 0,    /*!< input mode */
  DIR_OUT        /*!< output mode */
}GPIO_DIR_E ;

typedef struct {
	uint8_t port;	
	uint8_t dir ;
	uint8_t pr_mode;//GPIO_PR_E
	uint8_t drv;
}gpio_attribute_t;


typedef enum
{
  AFIO_EXTI_CH_0 = 0x00,     /*!< EXTI channel 0 */
  AFIO_EXTI_CH_1 = 0x04,     /*!< EXTI channel 1 */
  AFIO_EXTI_CH_2 = 0x08,     /*!< EXTI channel 2 */
  AFIO_EXTI_CH_3 = 0x0C,     /*!< EXTI channel 3 */
  AFIO_EXTI_CH_4 = 0x10,     /*!< EXTI channel 4 */
  AFIO_EXTI_CH_5 = 0x14,     /*!< EXTI channel 5 */
  AFIO_EXTI_CH_6 = 0x18,     /*!< EXTI channel 6 */
  AFIO_EXTI_CH_7 = 0x1C,     /*!< EXTI channel 7 */
  AFIO_EXTI_CH_8 = 0x20,     /*!< EXTI channel 8 */
  AFIO_EXTI_CH_9 = 0x24,     /*!< EXTI channel 9 */
  AFIO_EXTI_CH_10 = 0x28,    /*!< EXTI channel 10 */
  AFIO_EXTI_CH_11 = 0x2C,    /*!< EXTI channel 11 */
  AFIO_EXTI_CH_12 = 0x30,    /*!< EXTI channel 12 */
  AFIO_EXTI_CH_13 = 0x34,    /*!< EXTI channel 13 */
  AFIO_EXTI_CH_14 = 0x38,    /*!< EXTI channel 14 */
  AFIO_EXTI_CH_15 = 0x3C     /*!< EXTI channel 15 */
}AFIO_EXTI_CH_Enum ;
/**
 * @brief Enumeration of AFIO_MODE.
 */
typedef enum
{
 AFIO_MODE_DEFAULT = 0,   /*!< Default AFIO mode */
 AFIO_MODE_1,             /*!< AFIO mode 1 */
 AFIO_MODE_2,             /*!< AFIO mode 2 */
 AFIO_MODE_3,             /*!< AFIO mode 3 */
 AFIO_MODE_4,             /*!< AFIO mode 4 */
 AFIO_MODE_5,             /*!< AFIO mode 5 */
 AFIO_MODE_6,             /*!< AFIO mode 6 */
 AFIO_MODE_7,             /*!< AFIO mode 7 */
 AFIO_MODE_8,             /*!< AFIO mode 8 */
 AFIO_MODE_9,             /*!< AFIO mode 9 */
 AFIO_MODE_10,            /*!< AFIO mode 10 */
 AFIO_MODE_11,            /*!< AFIO mode 11 */
 AFIO_MODE_12,            /*!< AFIO mode 12 */
 AFIO_MODE_13,            /*!< AFIO mode 13 */
 AFIO_MODE_14,            /*!< AFIO mode 14 */
 AFIO_MODE_15             /*!< AFIO mode 15 */
}AFIO_MODE_Enum ;


/* Definitions of GPIO_PIN */
#define GPIO_PIN_0    0x0001   /*!< GPIO pin 0 selected */
#define GPIO_PIN_1    0x0002   /*!< GPIO pin 1 selected */
#define GPIO_PIN_2    0x0004   /*!< GPIO pin 2 selected */
#define GPIO_PIN_3    0x0008   /*!< GPIO pin 3 selected */
#define GPIO_PIN_4    0x0010   /*!< GPIO pin 4 selected */
#define GPIO_PIN_5    0x0020   /*!< GPIO pin 5 selected */
#define GPIO_PIN_6    0x0040   /*!< GPIO pin 6 selected */
#define GPIO_PIN_7    0x0080   /*!< GPIO pin 7 selected */
#define GPIO_PIN_8    0x0100   /*!< GPIO pin 8 selected */
#define GPIO_PIN_9    0x0200   /*!< GPIO pin 9 selected */
#define GPIO_PIN_10   0x0400   /*!< GPIO pin 10 selected */
#define GPIO_PIN_11   0x0800   /*!< GPIO pin 11 selected */
#define GPIO_PIN_12   0x1000   /*!< GPIO pin 12 selected */
#define GPIO_PIN_13   0x2000   /*!< GPIO pin 13 selected */
#define GPIO_PIN_14   0x4000   /*!< GPIO pin 14 selected */
#define GPIO_PIN_15   0x8000   /*!< GPIO pin 15 selected */
#define GPIO_PIN_ALL  0xFFFF   /*!< GPIO all pins selected */

/* Definitions of AFIO_PIN */
#define AFIO_PIN_0   (u32)0x00000001 /*!< AFIO pin 0 selected */
#define AFIO_PIN_1   (u32)0x00000002 /*!< AFIO pin 1 selected */
#define AFIO_PIN_2   (u32)0x00000004 /*!< AFIO pin 2 selected */
#define AFIO_PIN_3   (u32)0x00000008 /*!< AFIO pin 3 selected */
#define AFIO_PIN_4   (u32)0x00000010 /*!< AFIO pin 4 selected */
#define AFIO_PIN_5   (u32)0x00000020 /*!< AFIO pin 5 selected */
#define AFIO_PIN_6   (u32)0x00000040 /*!< AFIO pin 6 selected */
#define AFIO_PIN_7   (u32)0x00000080 /*!< AFIO pin 7 selected */
#define AFIO_PIN_8   (u32)0x00000100 /*!< AFIO pin 8 selected */
#define AFIO_PIN_9   (u32)0x00000200 /*!< AFIO pin 9 selected */
#define AFIO_PIN_10  (u32)0x00000400 /*!< AFIO pin 10 selected */
#define AFIO_PIN_11  (u32)0x00000800 /*!< AFIO pin 11 selected */
#define AFIO_PIN_12  (u32)0x00001000 /*!< AFIO pin 12 selected */
#define AFIO_PIN_13  (u32)0x00002000 /*!< AFIO pin 13 selected */
#define AFIO_PIN_14  (u32)0x00004000 /*!< AFIO pin 14 selected */
#define AFIO_PIN_15  (u32)0x00008000 /*!< AFIO pin 15 selected */
#define AFIO_PIN_ALL (u32)0x0000FFFF /*!< All AFIO pins selected */



extern int  gpio_request(unsigned gpio);
extern void gpio_free(unsigned gpio); 
extern int  gpio_request_array(int start, int num, int flags);
extern void gpio_free_array(int start, int num) ;
extern void gpio_set_value(unsigned gpio,int value) ;
extern int  gpio_get_value(unsigned gpio);
extern void gpio_to_exti(unsigned gpio);
extern int  gpio_set_afio(unsigned gpio, unsigned afio);
//extern int  gpio_attribute_cfg(gpio_attribute_t *io);
extern int  gpio_request_one(unsigned gpio, unsigned long flags) ;
#endif