//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "am_gpio.h"
#include "am_key.h"
#include "syscall_api.h"
static const single_gpio_key_map_t single_gpio_map[] =
{
  SINGLE_GPIO_KEY0,
  SINGLE_GPIO_KEY1,
  SINGLE_GPIO_KEY2,
  SINGLE_GPIO_KEY3,
  SINGLE_GPIO_KEY4,
  SINGLE_GPIO_KEY5,
  SINGLE_GPIO_KEY6,
  SINGLE_GPIO_KEY7
};

U8 key_single_init()
{
  int ret = -1;
  for( int i=0; i<SINGLE_GPIO_KEY_NUM; i++ )
  {
    ret = gpio_request_one(single_gpio_map[i].am_io,single_gpio_map[i].input_gpio_state);
    
    if(ret != -1)
    {
      ;
    }
    else
    {
      printf("key io not ok\r\n");
    }
  }  
  return 0;
}

U8 key_single_exit()
{
  for( int i=0; i<SINGLE_GPIO_KEY_NUM; i++ )
  {
    gpio_free(single_gpio_map[i].am_io);
  }     
  return 0;
}

/******************************************************************************/
/*!
* \par  Description:
*      单GPIO按键支持多按键，要全部检测完才返回
* \retval       int: 键值
*******************************************************************************/
U32 get_single_keyval(void)
{
  U8 i = 0;
  U8 gpio_state;
  U32 keyval = KEY_NULL;
  
  //read all gpio
  for( i=0; i<sizeof(single_gpio_map)/sizeof(single_gpio_key_map_t); i++ )
  {
    if(single_gpio_map[i].keyval==KEY_NULL)
    {
      continue;
    }
    else
    {
      gpio_state = gpio_get_value(single_gpio_map[i].am_io);
      if ( gpio_state ==  single_gpio_map[i].input_gpio_state)
      {
        keyval =(keyval | single_gpio_map[i].keyval);
      }            
    }
  }
  
  return keyval;
}
