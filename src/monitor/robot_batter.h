#ifndef ROBOT_BATTER_H
#define ROBOT_BATTER_H

#include "am_type.h"

typedef struct
{
  u16   voltage;           // mV
  u16   capacity;          // mAh
  u16   remaining_charge;  // mAh
  s16   temperature;       // C
  u16   charger_voltage;   // mV
  u16   charging_current;  // mA 
  u16   charging_state;
  u8    last_status;
} BatteryStatus;

#define CHARGE_NULL 0
#define DOCKING   1
#define JACKING   2

typedef enum
{
    DOCK_NONE,
    DOCK_ING,
    DOCK_KIDNAP,
} DockingKidnap;
void robot_docking_kidnap_monitor(BatteryStatus *battery_status);
void robot_battery_monitor(void);
void robot_battery_init(void);
void robot_battery_monitor_routine(void);
u8 charging_detect(void);
u8 battery_is_low(void);
#endif