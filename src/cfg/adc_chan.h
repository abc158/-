#ifndef _ADC_CHAN_H_
#define _ADC_CHAN_H_

typedef enum
{
  /*以下表格不能更改lyy*/
  /* Chnl  SignalName */
  /*  0 */ ADC_CHAN_CLIFF_LEFT , 
           ADC_CHAN_CLIFF_FRONTLEFT,
           ADC_CHAN_CLIFF_RIGHT,
           ADC_CHAN_REAR_CLIFF_LEFT,
           ADC_CHAN_REAR_CLIFF_RIGHT,           
           ADC_CHAN_CLIFF_FRONTRIGHT,
  /*  6 */ ADC_CHAN_LT_RIGHT  , 
  /*  7 */ ADC_CHAN_LT_CENTERLEFT ,  
  /*  8 */ ADC_CHAN_LT_CENTERRIGHT  ,    
  /*  9 */ ADC_CHAN_LT_FRONTRIGHT,    
   /*10 */ ADC_CHAN_LT_FRONTLEFT,
  /* 11 */ ADC_CHAN_LT_LEFT,
  /* 12 */ ADC_CHAN_VAC_CURENT,
  /* 13 */ ADC_CHAN_BATTERY_THERMISTOR   ,
  /* 14 */ ADC_CHAN_BATTERY_CURRENT     ,
  /* 15 */ ADC_CHAN_BATTERY_VOLTAGE,
  /* 16 */ ADC_CHAN_LEFT_MOTOR_CURRENT  ,
  /* 17 */ ADC_CHAN_RIGHT_MOTOR_CURRENT ,
  /* 18 */ ADC_CHAN_MAIN_BRUSH_CURRENT  ,
  /* 19 */ ADC_CHAN_SIDE_BRUSH_CURRENT,   
  /* 20 */ ADC_CHAN_BIN_LEFT ,
  /* 21 */ ADC_CHAN_BIN_RIGHT,
  /* 22 */ HAL_REV1,
  /* 23 */ HAL_REV2,
  /* 24 */ HAL_REV3,
  /* 25 */ HAL_REV4, 
  /* 26 */ ADC_CHAN_MAX
} ADC_ID_E;

typedef struct
{
  U8 logic_chan;
  U8 phy_chan;
}adc_chan_t;


#define AM380S_ADC_CHAN_INIT(name,chan) \
  {name,chan} ,

/*不同的方案修改下面表格 chan 通道数即可，下面排序不能变，只能修改数字，比如：ADC_CHAN_CLIFF_LEFT 0， 如果不是0通道，则把0改为其他数字。 lyy*/
#define ROBOT_ADCCHANDEF(m)      \
  m(ADC_CHAN_CLIFF_LEFT               ,0  )\
  m(ADC_CHAN_CLIFF_FRONTLEFT          ,1  )\
  m(ADC_CHAN_CLIFF_RIGHT              ,4  )\
  m(ADC_CHAN_REAR_CLIFF_LEFT          ,2  )\
  m(ADC_CHAN_REAR_CLIFF_RIGHT         ,3  )\
  m(ADC_CHAN_CLIFF_FRONTRIGHT         ,5  )\
  m(ADC_CHAN_LT_RIGHT          ,6  )\
  m(ADC_CHAN_LT_CENTERLEFT     ,7  )\
  m(ADC_CHAN_LT_CENTERRIGHT    ,8  )\
  m(ADC_CHAN_LT_FRONTRIGHT     ,9  )\
  m(ADC_CHAN_LT_FRONTLEFT      ,10 )\
  m(ADC_CHAN_LT_LEFT           ,11 )\
  m(ADC_CHAN_VAC_CURENT               ,12 )\
  m(ADC_CHAN_BATTERY_THERMISTOR       ,13 )\
  m(ADC_CHAN_BATTERY_CURRENT          ,14 )\
  m(ADC_CHAN_BATTERY_VOLTAGE          ,15 )\
  m(ADC_CHAN_LEFT_MOTOR_CURRENT       ,16 )\
  m(ADC_CHAN_RIGHT_MOTOR_CURRENT      ,17 )\
  m(ADC_CHAN_MAIN_BRUSH_CURRENT       ,18 )\
  m(ADC_CHAN_SIDE_BRUSH_CURRENT       ,19 )\
  m(ADC_CHAN_BIN_LEFT                 ,20 )\
  m(ADC_CHAN_BIN_RIGHT                ,21 )\
  m(HAL_REV1                         ,22 )\
  m(HAL_REV2                         ,23 )\
  m(HAL_REV3                         ,24 )\
  m(HAL_REV4                         ,25 )\

#endif