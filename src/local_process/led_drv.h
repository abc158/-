#ifndef __LED_DRV___
#define __LED_DRV___

#define DISPLAY_COLON_ON 0x80
#define DISPLAY_COLON_OFF 0x0
#define DISPLAY_CLOCK_ITEM_ON 0x80
#define DISPLAY_CLOCK_ITEM_OFF 0x0
#define DISPLAY_CLEAN_MODE_NORMAL 0x04
#define DISPLAY_CLEAN_MODE_SPOT 0x40
#define DISPLAY_CLEAN_MODE_WALL_FOLLOW 0x08
#define DISPLAY_CLEAN_MODE_RANDOM 0x20
#define DISPLAY_CLEAN_MODE_NONE 0
#define DISPLAY_BATTERY_LEVEL_1 0x10
#define DISPLAY_BATTERY_LEVEL_2 0x12
#define DISPLAY_BATTERY_LEVEL_3 0x13
#define DISPLAY_BATTERY_OFF 0x00
#define DISPLAY_COLON_CLEAR 0x7f
#define DISPLAY_CLOCK_ITEM_CLEAR 0x7f
#define DISPLAY_CLEAN_MODE_CLEAR 0x93
#define DISPLAY_BATTERY_CLEAR 0xec


typedef enum
{
  clean_mode_normal,
  clean_mode_spot,
  clean_mode_wall_follow,
  clean_mode_random,
  clean_mode_none
}display_clean_mode_e;

typedef enum
{
  battery_level_0,
  battery_level_1,
  battery_level_2,
  battery_level_3
}display_battery_level_e;

extern void led_drv_init(void);
extern void led_drv_test(void);
extern void led_display_num(u8 index, char letter);
extern void led_display_colon(u8 enable);
extern void led_display_clock_item(u8 enable);
extern void led_display_clean_mode(u8 mode);
extern void led_display_battery(u8 level);
extern void led_display_update(void);
extern void ledc_sniff(void);
#endif
