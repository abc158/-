#ifndef __GPIO_INIT_H
#define __GPIO_INIT_H

typedef struct
{
	uint8_t dir;
	uint8_t port;
	uint8_t start;
	uint8_t width;
	uint8_t pr_mode;

} IO_PIN_INIT_CFG;

#define AMICRO_R10_IODEF(m)                                             \
	/* Name                   IO_PIN_CFG initializer             */  \
        m( 0x55 ,           0xaa ,  0 , 0,0 )   \
	/* Port 0 A  Direction   Port Strt Wdth*/\
	m( GPIO_DIR_IN ,     0 ,    0 ,16 ,GPIO_PR_DISABLE )   \
	/* Port 0 B  */\
	m( GPIO_DIR_IN ,     1 ,  0 ,   6 ,GPIO_PR_DISABLE )   \
	m( GPIO_DIR_IN ,     1 ,  6 ,   1 ,GPIO_PR_DISABLE )   \
	m( GPIO_DIR_IN ,     1 ,  7 ,   1 ,GPIO_PR_DISABLE )   \
	m( GPIO_DIR_OUT ,    1 ,  8 ,   1 ,GPIO_PR_UP     )   \
	m( GPIO_DIR_IN ,     1 ,  9 ,   1 ,GPIO_PR_DISABLE )   \
	m( GPIO_DIR_IN ,     1 ,  10 ,  1 ,GPIO_PR_DISABLE )   \
        m( GPIO_DIR_OUT ,    1 ,  11 ,  1 ,GPIO_PR_UP      )   \
	m( GPIO_DIR_IN ,     1 ,  12 ,  1 ,GPIO_PR_DISABLE )   \
	m( GPIO_DIR_IN ,     1 ,  13 ,  1 ,GPIO_PR_DISABLE )   \
	m( GPIO_DIR_IN ,     1 ,  14 ,  1 ,GPIO_PR_DISABLE )   \
	m( GPIO_DIR_IN ,     1 ,  15 ,  1 ,GPIO_PR_DISABLE )   \
        /*Port 0 C*/\
	m( GPIO_DIR_IN ,     2 ,  0 ,   1 ,GPIO_PR_UP       )   \
        m( GPIO_DIR_OUT ,     2 ,  1 ,   1 ,GPIO_PR_UP       )   \
	m( GPIO_DIR_IN ,     2 ,  2 ,   1 ,GPIO_PR_DISABLE  )  \
	m( GPIO_DIR_IN ,     2 ,  3 ,   1 ,GPIO_PR_DISABLE  )\
        m( GPIO_DIR_IN ,     2 ,  4 ,   1 ,GPIO_PR_DISABLE  ) \
	m( GPIO_DIR_OUT ,    2 ,  5 ,   1 ,GPIO_PR_UP       )   \
	m( GPIO_DIR_OUT ,    2 ,  6 ,   1 ,GPIO_PR_DOWN     )  \
	m( GPIO_DIR_IN ,     2 ,  7 ,   1 ,GPIO_PR_DISABLE  )  \
	m( GPIO_DIR_IN ,     2 ,  8 ,   1 ,GPIO_PR_DISABLE  )\
	m( GPIO_DIR_IN ,     2 ,  9 ,   1 ,GPIO_PR_DISABLE  ) \
	m( GPIO_DIR_OUT ,    2 ,  10 ,  1 ,GPIO_PR_UP       ) \
	m( GPIO_DIR_OUT ,     2 ,  11 ,  1 ,GPIO_PR_UP       )   \
	m( GPIO_DIR_IN ,     2 ,  12 ,  1 ,GPIO_PR_UP       ) \
	m( GPIO_DIR_IN ,     2 ,  13 ,  1 ,GPIO_PR_DISABLE  ) \
	m( GPIO_DIR_OUT ,    2 ,  14 ,  2 ,GPIO_PR_UP       )\
	/* Port 0 D  */                                    \
	m( GPIO_DIR_OUT ,    3 ,  0 ,   1 ,GPIO_PR_UP       )/*led seg6*/\
	m( GPIO_DIR_OUT ,    3 ,  1 ,   1 ,GPIO_PR_DOWN     )/*led seg7*/\
	m( GPIO_DIR_OUT ,    3 ,  2 ,   1 ,GPIO_PR_UP       )/*led dig0*/\
	m( GPIO_DIR_OUT ,    3 ,  3 ,   1 ,GPIO_PR_UP       )/*led dig1*/\
	m( GPIO_DIR_OUT ,    3 ,  4 ,   1 ,GPIO_PR_UP       )/*cliff sel*/   \
	m( GPIO_DIR_IN ,     3 ,  5 ,   1 ,GPIO_PR_DISABLE  )/*cliff ir en*/  \
	m( GPIO_DIR_OUT ,    3 ,  6 ,   1 ,GPIO_PR_DOWN     )/*spen*/\
	m( GPIO_DIR_OUT ,    3 ,  7 ,   1 ,GPIO_PR_UP       )/*led dig3*/\
	m( GPIO_DIR_IN ,     3 ,  8 ,   1 ,GPIO_PR_DISABLE   )/*led dig4*/\
	m( GPIO_DIR_OUT ,    3 ,  9 ,   1 ,GPIO_PR_UP       )/*led dig5*/\
	m( GPIO_DIR_IN ,     3 ,  10 ,  1 ,GPIO_PR_UP        )/*bin left en*/  \
	m( GPIO_DIR_OUT ,    3 ,  11 ,  1 ,GPIO_PR_UP       )/*bin right en*/ \
	m( GPIO_DIR_IN ,     3 ,  12 ,  1 ,GPIO_PR_UP        )/*binled left drv*/\
	m( GPIO_DIR_OUT ,    3 ,  13 ,  1 ,GPIO_PR_UP       )/*binled right drv*/\
        m( GPIO_DIR_OUT ,    3 ,  14 ,  2 ,GPIO_PR_DOWN       )/*main/side brush pwm*/\
        \
	/*port 0 E*/						\
	m( GPIO_DIR_OUT ,   4 ,  0 ,   1 ,GPIO_PR_UP       )/*motor sleep*/   \
	m( GPIO_DIR_IN  ,   4 ,  1 ,   1 ,GPIO_PR_DISABLE  )/*uart debug rx*/\
	m( GPIO_DIR_OUT ,   4 ,  2 ,   1 ,GPIO_PR_UP       )/*uart debug tx*/\
	m( GPIO_DIR_OUT  ,  4 ,  3 ,   1 ,GPIO_PR_DOWN     )/*side brush ctl*/\
	m( GPIO_DIR_IN  ,   4 ,  4 ,   1 ,GPIO_PR_DISABLE  )/*motor  fault*/\
	m( GPIO_DIR_OUT ,   4 ,  5 ,   1 ,GPIO_PR_DOWN     )/*main brush ctl*/ \
	m( GPIO_DIR_OUT ,   4 ,  6 ,   1 ,GPIO_PR_UP       )/*motor reset */   \
	m( GPIO_DIR_OUT ,   4 ,  7 ,   1 ,GPIO_PR_UP       )/*main to ui */   \
	m( GPIO_DIR_IN  ,   4 ,  8 ,   1 ,GPIO_PR_DISABLE  )/*ui to main */  \
	m( GPIO_DIR_IN  ,   4 ,  9 ,   1 ,GPIO_PR_DISABLE  )/*reset to ui */\
	m( GPIO_DIR_OUT ,   4 ,  10 ,  1 ,GPIO_PR_DISABLE       )/*light pwm */\
	m( GPIO_DIR_IN  ,   4 ,  11 ,  1 ,GPIO_PR_DISABLE  )/*vac pwm */ \
	m( GPIO_DIR_OUT ,    4 ,  12 ,  1 ,GPIO_PR_DISABLE  )/*int to ui */\
	m( GPIO_DIR_OUT ,   4 ,  13 ,  1 ,GPIO_PR_DISABLE     )/*int to main */\
	m( GPIO_DIR_IN  ,   4 ,  14 ,  1 ,GPIO_PR_DISABLE  )/*bump left */\
	m( GPIO_DIR_IN  ,   4 ,  15 ,  1 ,GPIO_PR_DISABLE  )/*bump right */\
	/* Port 0 F  */                                          \
	m( GPIO_DIR_IN ,   5 ,  0 ,   1 ,GPIO_PR_DISABLE     )/*audio en */\
	m( GPIO_DIR_IN ,    5 ,  1 ,   1 ,GPIO_PR_DISABLE  )/*ir remote / ir a /ir b*/   \
  m( GPIO_DIR_IN ,    5 ,  2 ,   1 ,GPIO_PR_DISABLE  )/*ir remote / ir a /ir b*/   \
  m( GPIO_DIR_IN ,    5 ,  3 ,   1 ,GPIO_PR_DISABLE  )/*ir remote / ir a /ir b*/   \
	m( GPIO_DIR_OUT ,   5 ,  4 ,   1 ,GPIO_PR_DISABLE  )/*spi1 ss */\
	m( GPIO_DIR_OUT ,   5 ,  5 ,   1 ,GPIO_PR_DISABLE  )/* spi1 sck*/ \
	m( GPIO_DIR_OUT ,   5 ,  6 ,   1 ,GPIO_PR_DISABLE  )/* spi1 mosi */ \
	m( GPIO_DIR_IN ,    5 ,  7 ,   1 ,GPIO_PR_DISABLE  )/*spi1 miso*/  \
	m( GPIO_DIR_OUT ,   5 ,  8 ,   1 ,GPIO_PR_UP       )/*CHARGER EN */ \
	m( GPIO_DIR_OUT ,   5 ,  9 ,   1 ,GPIO_PR_UP       )/*CURRENT CAL */ \
	m( GPIO_DIR_OUT ,   5 ,  10 ,  1 ,GPIO_PR_UP       )/*WATCHDOG KICKER */ \
	m( GPIO_DIR_OUT ,   5 ,  11 ,  1 ,GPIO_PR_UP       )/* DCDC DISABLE */ \
	m( GPIO_DIR_OUT ,   5 ,  12 ,  1 ,GPIO_PR_DOWN     )/* MOTOR R CTL */ \
	m( GPIO_DIR_OUT ,   5 ,  13 ,  1 ,GPIO_PR_UP       )/* SPI SCK */   \
	m( GPIO_DIR_OUT ,   5 ,  14 ,  1 ,GPIO_PR_DOWN     )/* SPI MOSI */   \
	m( GPIO_DIR_IN ,    5 ,  15 ,  1 ,GPIO_PR_DISABLE  )/* SPI MISO */ \
	/* Port 0 G  */                                                  \
	m( GPIO_DIR_IN ,    6 ,  0 ,   1 ,GPIO_PR_DISABLE )/* MOTOR R ENCODE PID1_I*/ \
	m( GPIO_DIR_OUT ,   6 ,  1 ,   1 ,GPIO_PR_UP      )/* MOTOR R PWM    PID1_O*/  \
	m( GPIO_DIR_OUT ,   6 ,  2 ,   1 ,GPIO_PR_UP     )/*5V*/  \
	m( GPIO_DIR_OUT  ,  6 ,  3 ,   1 ,GPIO_PR_UP     )/*3.3V*/ \
        m( 0  ,             7 ,  0 ,   0 ,0 )/*end condition*/

#define AMCIRO_R10_CFG_INST(dir, port, start, width, pr_mode) \
  { dir, port, start, width, pr_mode} ,

#endif /* __GPIO_INIT_H */

