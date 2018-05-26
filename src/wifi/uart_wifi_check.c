#include <ui-config.h>
#include "ui-commands.h"
#include "am_date_base.h"
#include "am_config_macro.h"
#include "time.h"
#include "lib.h"
#include "local_key_check.h"
#include "syscall_api.h"

#include "am_date_base.h"
#include "ui-commands.h"
#include "display.h"

#if defined(USE_WIFI_DEMO_1)
#include "wifi_demo1/simwifi.h"
#elif defined(USE_WIFI_DEMO_2)
#include "wifi_demo2/simwifi_demo2.h"
#endif
#include "act.h"

#if defined(USE_UART_WIFI)
static U8 key_check[4];

static cmd_state_t wifi_uart_cmd_state[UART_CMD_MAX]=
{	
    {KEY_NULL, UART_START,       KEY_CLEAN},
    {KEY_NULL, UART_DOCK,        KEY_DOCK},
    {KEY_NULL, UART_STOP,        KEY_CLEAN},//
    {KEY_NULL, UART_LEFT,        KEY_LEFT},
    {KEY_NULL, UART_RIGHT,       KEY_RIGHT},
    {KEY_NULL, UART_FORWARD,     KEY_FORWARD},
    {KEY_NULL, UART_BACK,        KEY_BACKWARD},
    {KEY_NULL, UART_SPOT,        KEY_SPOT},
    {KEY_NULL, UART_QUIT,        KEY_POWER},
    {KEY_NULL, UART_POWER_ON,    KEY_POWER},
    {KEY_NULL, UART_POWER_DOWN,  KEY_POWER},
    {KEY_NULL, UART_WALL_FOLLOW, KEY_WALLFLOW},
    {KEY_NULL, UART_ENHANCE,     KEY_WALLFLOW},
        
};


void uart_cmd_ext_handle(void)
{
    U8 i;
    for(i=0;i<4;i++)
    {
        if(key_check[i] >= 32)
        {
            wifi_uart_cmd_state[i+3].state = KEY_SHORT_UP;
            key_check[i] = 0;
        }
        else if((key_check[i]>=1)&&(key_check[i]<32))
        {
            key_check[i]++;
        }
    }
}

//64Hz call
U16 get_tmp_cmd(void)
{

  return 0;
}

void wifi_uart_cmd_rounte(void)
{
    int i = 0;
    #if defined(USE_WIFI_DEMO_1) || defined(USE_WIFI_DEMO_2)
    U16 cmd = get_uart_cmd();
    #elif defined(USE_UART_WIFI)
    U16 cmd = get_tmp_cmd();
    #endif
    for(i = 0;i<UART_CMD_MAX;i++)
    {
      if(cmd == wifi_uart_cmd_state[i].cmd_code)
      {
          if((i==3)||(i==4)||(i==5)||(i==6)) //方向键处理
          {
              if(key_check[(i-3)] > 0)
              {
                  wifi_uart_cmd_state[i].state = PRESS_NULL;
              }
              else
              {
                  wifi_uart_cmd_state[i].state = KEY_DOWN;
              }
              key_check[0] = 0;
              key_check[1] = 0;
              key_check[2] = 0;
              key_check[3] = 0;
              key_check[(i-3)] = 1;
              #if defined(USE_WIFI_DEMO_2)
              if((get_ui_state() == UI_ENTER_SLEEPING))
              {
                  set_ui_state(UI_IDLE);
                 // return 0;
              }
              #endif
          }
          else
          {
              key_check[0] = 0;
              key_check[1] = 0;
              key_check[2] = 0;
              key_check[3] = 0;
             // printf("uart_cmd i=%d\r\n",i);
              wifi_uart_cmd_state[i].state = KEY_SHORT_UP;
          }
           
      }
      else
      {
          wifi_uart_cmd_state[i].state = PRESS_NULL;
      }
    }
    uart_cmd_ext_handle();
}



cmd_state_t *get_wifi_uart_cmd_state(void)
{
    return &wifi_uart_cmd_state[0];
}

#endif
