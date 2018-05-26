#ifndef __DISPLAY__
#define __DISPLAY__

#define FAST_TIME 18
#define SLOW_TIME 50

#define CLEAN_LED 1
#define DOCK_LED  2
#define SPOT_LED  3
#define ALL_LED   4
#define RED_LED   CLEAN_LED
#define GREEN_LED SPOT_LED
#define ORANGE_LED ALL_LED

#define LED_FLASH_TIME 120
#define HALF_LED_FLASH_TIME (LED_FLASH_TIME/2)

#define BAT_FLASH_TIME 120
#define HALF_BAT_FLASH_TIME (BAT_FLASH_TIME/2)

#define LED_GPIO_NUM 3

extern void _display_init(void);
extern void _display_exit(void);
extern void display_routine(void);
extern void reset_led(U8 led_id);
extern void set_led(U8 led_id);
extern void set_display_power_up_flag(U8 state);


#endif
