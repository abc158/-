#ifndef UNIVRSAL_WHEEL_H
#define UNIVRSAL_WHEEL_H
#include "am_type.h"
#include "am_config_macro.h"

#ifndef M_PI_F
    #define M_PI_F 3.14159265358979323846f
#endif

#define STASIS_DIAMETER_IN_MM 19.5f
#define STASIS_RUN_LENTH_IN_MM (STASIS_DIAMETER_IN_MM*M_PI_F)
#define STASIS_TRIGGER_NORMAL_LENTH  STASIS_RUN_LENTH_IN_MM*1.5f
#define STASIS_TRIGGER_HIGH_LENTH    STASIS_RUN_LENTH_IN_MM*1.9f
#define STASIS_SET_HIGH_THROD        STASIS_RUN_LENTH_IN_MM*3.5f
#define STASIS_ALWAYS_SET_THROD      STASIS_RUN_LENTH_IN_MM*5.0f

typedef struct
{
	float x;
	float y;
}AM_xy;

typedef struct
{
   AM_xy xy;
   float angle;
} AM_Pose;

#define TICKS_PER_MM    (WHEEL_CODE_RATIO / (M_PI_F * WHEEL_DIA))

/*change ticks/s to  mm/s */
static inline float ticks_to_mm(s32 ticks)
{
    return ((float)ticks / TICKS_PER_MM);
}

extern  void  robot_stasis_test_clear(void);
extern  U32   robot_stasis_test_get(void);
extern  int   universal_wheel_monitor(void *arg);
extern  void  robot_universl_wheel_init(void);
extern  void  robot_universl_wheel_exit(void);
extern  int   universal_wheel_count(void);
extern void robot_universal_wheel_init(void);

#endif