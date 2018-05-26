#ifndef CURRENT_H
#define CURRENT_H

s16 battery_voltage_average(void);
u16 check_battery_voltage(u16 vol_adc);
u16 battery_temperature_average(void);
float battery_temperature_get(void);
#endif