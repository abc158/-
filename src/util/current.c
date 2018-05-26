#include "am_type.h"
#include "adc_chan.h"
#include "syscall_api.h"

volatile uint16_t main_brush_current = 0;
volatile uint16_t side_brush_current = 0;
volatile uint16_t left_motor_current = 0;
volatile uint16_t right_motor_current = 0;
volatile uint16_t battery_line_current = 0;
volatile uint16_t vaccum_motor_current = 0;
volatile uint16_t battery_voltage = 0;
volatile uint16_t battery_temperature =0;

uint16_t last_main_brush_current = 0;
uint16_t last_side_brush_current = 0;
uint16_t last_left_motor_current = 0;
uint16_t last_right_motor_current = 0;
uint16_t last_battery_line_current = 0;
uint16_t last_vaccum_motor_current = 0;
uint16_t last_battery_voltage      = 0;
uint16_t last_battery_temperature  = 0;

static U8 initialized;

void current_filter_routine(void)
{
    uint16_t tmp16;
    tmp16 = get_adc_chan_val(ADC_CHAN_MAIN_BRUSH_CURRENT);
    main_brush_current = tmp16/10 + last_main_brush_current*9/10;
    last_main_brush_current = main_brush_current;

    tmp16 = get_adc_chan_val(ADC_CHAN_SIDE_BRUSH_CURRENT);
    side_brush_current = tmp16/10 + last_side_brush_current*9/10;
    last_side_brush_current = side_brush_current;

    tmp16 = get_adc_chan_val(ADC_CHAN_LEFT_MOTOR_CURRENT);
    left_motor_current = tmp16/10 + last_left_motor_current*9/10;
    last_left_motor_current = left_motor_current;

    tmp16 = get_adc_chan_val(ADC_CHAN_RIGHT_MOTOR_CURRENT);
    right_motor_current = tmp16/10 + last_right_motor_current*9/10;
    last_right_motor_current = right_motor_current;

    tmp16 = get_adc_chan_val(ADC_CHAN_BATTERY_CURRENT);
    battery_line_current = tmp16/10 + last_battery_line_current*9/10;
    last_battery_line_current = battery_line_current;

    tmp16 = get_adc_chan_val(ADC_CHAN_VAC_CURENT);
    vaccum_motor_current = tmp16/10 + last_vaccum_motor_current*9/10;
    last_vaccum_motor_current = vaccum_motor_current;
    
    tmp16 = get_adc_chan_val(ADC_CHAN_BATTERY_VOLTAGE);
    battery_voltage = tmp16/10 + last_battery_voltage*9/10;
    last_battery_voltage = battery_voltage;    
 
    tmp16 = get_adc_chan_val(ADC_CHAN_BATTERY_VOLTAGE);
    battery_temperature = tmp16/10 + last_battery_temperature*9/10;
    last_battery_temperature = battery_temperature;       
    
}

void robot_current_init(void)
{
    if(!initialized)
    {
        timer_task_register(current_filter_routine, 5, TRUE);//5ms / per calc
        initialized = 1;
    }
}

u16 battery_voltage_average(void)
{
  return last_battery_voltage;
}

u16 battery_temperature_average(void)
{
  return last_battery_temperature;
}

/****************************************************************
*Function   :  check_battery_voltage
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  判断当前电压是否超过vol_adc
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              vol_adc : adc 值， 用来和当前电压对比
*Output     :  无
*Return     :  返回 1：大于adc，  0：小于等于
*Others     :  可以在本地实现同等功能函数
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U16   check_battery_voltage(U16 vol_adc);
u16 check_battery_voltage(u16 vol_adc)
{
  return (battery_voltage_average()>vol_adc);
}

/****************************************************************
*Function   :  battery_temperature_get
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取电池温度的热敏电阻值
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回，电阻值 （单位：欧）
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
float battery_temperature_get(void)
{
  volatile uint16_t t = battery_temperature_average();//hal_analog_input_get(ADC_CHAN_BATTERY_THERMISTOR);
  float v = (float)((float)t/4096)*3.3f;
  float x = v/3.3f;
  float r =(((47*x)/(1-x)));
  return r;
}
