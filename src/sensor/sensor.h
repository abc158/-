#ifndef _TOUCH_
#define _TOUCH_

#include <stdio.h>
#include "stdint.h"
#include "am_type.h"
#include "am_robot_type.h"

/////////////////////////////////////////////////
/*这里的信息不能修改，要和主板保持一致，lyy*/
#define IR_SENSOR_NUM 16

typedef enum {
  CLIFF_RIGHT,
  CLIFF_FRONTRIGHT,
  CLIFF_FRONTLEFT,
  CLIFF_LEFT,
  CLIFF_REAR_RIGHT,
  CLIFF_REAR_LEFT,
  CLIFF_REV1,
  CLIFF_REV2,
  LT_CENTERRIGHT,
  LT_FRONTLEFT,
  LT_RIGHT,
  LT_LEFT,
  LT_FRONTRIGHT,
  LT_CENTERLEFT, 
  LT_CENTERLEFT_L,
  LT_CENTERRIGHT_L,
}SENSOR_E;
/////////////////////////////////////////////////////
#define ADC_BASE_ADDR  syscall_irq->get_adc()

extern U8  robot_is_cliff(U8 index);
extern S16 robot_signal_distance(U8 index);
extern S16 robot_rear_lt_distance(U8 index);
extern u8  robot_is_lighttouch(u8 index);
extern u16 hal_isr(void);
extern void robot_close_sensor_led(void);
extern void robot_sensor_init(void);
extern void sensor_handle(void);
extern void sensor_gather(void);
extern void set_lighttouch_enable(u8 en);
extern void set_cliff_enable(u8 en);
extern void set_cliff_threshold(SENSOR_E chan, int val);
extern void set_cliff_enable(u8 en);
extern void reset_cliff_threshold(void);
extern void robot_ir_detection_init(void);
extern void robot_sensor_gather_start(u8 en);
extern u16 mid_filter(s16* data_array, u16 new_data);
extern void turn_off_touch_bump(void);
extern void turn_on_touch_bump(void);
extern int touch_bump_run(void);
#endif