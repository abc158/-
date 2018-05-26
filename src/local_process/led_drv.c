#include "syscall_api.h"
#include "am_gpio.h"
#include "led_drv.h"

#define LED_DATA_HIGH() gpio_set_value(AM_IO_LED_DIN, 1)
#define LED_DATA_LOW() gpio_set_value(AM_IO_LED_DIN, 0)
#define LED_CLK_HIGH() gpio_set_value(AM_IO_LED_CLK, 1)
#define LED_CLK_LOW() gpio_set_value(AM_IO_LED_CLK, 0)
#define LED_CS_HIGH() gpio_set_value(AM_IO_LED_STB, 1)
#define LED_CS_LOW() gpio_set_value(AM_IO_LED_STB, 0)

u8 display_tabel[]=
{
  0x3f,//0
  0x06,//1
  0x5b,//2
  0x4f,//3
  0x66,//4
  0x6d,//5
  0x7d,//6
  0x07,//7
  0x7f,//8
  0x6f,//9
  0x77,//A
  0x77,//B
  0x77,//C
  0x77,//D
  0x77,//E
  0x77,//F
  0x77,//G
  0x77,//H
  0x77,//I
  0x77,//J
  0x77,//K
  0x77,//L
  0x77,//M
  0x77,//N
  0x77,//O
  0x77,//P
  0x77,//Q
  0x77,//R
  0x77,//S
  0x77,//T
  0x77,//U
  0x77,//V
  0x77,//W
  0x77,//X
  0x77,//Y
  0x77 //Z
};

u8 save_buf[5] = {0, 0, 0, 0, 0};
u8 display_colon = DISPLAY_COLON_OFF;
u8 display_clock_item = DISPLAY_CLOCK_ITEM_OFF;

void delay_us(u16 delay)
{
  u16 temp;
  
  temp = delay*10;
  while(temp)
  {
    temp--;
  }
}


void led_drv_send_data(u8* addr, u8 len)
{
  u8 data, i, j;
  
  //LED_CS_LOW();
  //delay_us(1);
  
  for (i=0; i<len; i++)
  {   
    data = *(addr+i);
    //AM_DEBUG_INFO("b:");
    for (j=0; j<8; j++)
    {
      LED_CLK_LOW();
      
      if (data>>j & 0x01)
      {
        LED_DATA_HIGH();
        //AM_DEBUG_INFO("1");
      }
      else
      {
        LED_DATA_LOW();
        //AM_DEBUG_INFO("0");
      }
      delay_us(1);
      LED_CLK_HIGH();
      delay_us(1);
    }
  }
  
  //LED_CS_HIGH();
  //delay_us(1);
}

void led_drv_send_cmd(u8 command)
{
  u8 cmd_data[1];
  
  cmd_data[0] = command;
  
  LED_CS_LOW();
  delay_us(1);
  led_drv_send_data(cmd_data, 1);
  if (command&0xc0 != 0xc0)
  {
    LED_CS_HIGH();
    delay_us(1);
  }
}


void led_display_all_off(void)
{     
  memset(save_buf, 0, sizeof(save_buf));
  display_colon = DISPLAY_COLON_OFF;  
  display_clock_item = DISPLAY_CLOCK_ITEM_OFF;
}


void led_display_num(u8 index, char letter)
{
  if (index <= 3)
  { 
    if (letter <= 9)
    {
      save_buf[index] = display_tabel[letter];            
    }  
    else if ((letter >= 'A') && (letter <= 'Z'))
    {
      save_buf[index] = display_tabel[letter-'A'+10];                 
    }    
    if(index == 1 || index == 2)
    {
      save_buf[index] |= display_colon;
    }  
  }        
}


void led_display_colon(u8 enable)
{
  if (enable == 1)
  {  	
    display_colon = DISPLAY_COLON_ON;
  }
  else
  {
    display_colon = DISPLAY_COLON_OFF;
  } 
  save_buf[1] &= DISPLAY_COLON_CLEAR;
  save_buf[2] &= DISPLAY_COLON_CLEAR;
  save_buf[1] |= display_colon;
  save_buf[2] |= display_colon;
}


void led_display_clock_item(u8 enable)
{
  save_buf[4] &= DISPLAY_CLOCK_ITEM_CLEAR;
  if (enable == 1)
  {  	
    save_buf[4] |= DISPLAY_CLOCK_ITEM_ON;
  }
  else
  {
    save_buf[4] |=  DISPLAY_CLOCK_ITEM_OFF;
  } 
}


void led_display_clean_mode(u8 mode)
{
  save_buf[4] &= DISPLAY_CLEAN_MODE_CLEAR;
  switch (mode)
  {
  case clean_mode_normal:
    save_buf[4] |= DISPLAY_CLEAN_MODE_NORMAL;
    break;
    
  case clean_mode_spot:
    save_buf[4] |= DISPLAY_CLEAN_MODE_SPOT;
    break;
    
  case clean_mode_wall_follow:
    save_buf[4] |= DISPLAY_CLEAN_MODE_WALL_FOLLOW;
    break;
    
  case clean_mode_random:
    save_buf[4] |= DISPLAY_CLEAN_MODE_RANDOM;
    break;
    
  default:
    save_buf[4] |= DISPLAY_CLEAN_MODE_NONE;
    break;
  }   
}


void led_display_battery(u8 level)
{
  save_buf[4] &= DISPLAY_BATTERY_CLEAR;
  switch (level)
  {
  case battery_level_1:
    save_buf[4] |= DISPLAY_BATTERY_LEVEL_1;
    break;
    
  case battery_level_2:
    save_buf[4] |= DISPLAY_BATTERY_LEVEL_2;
    break;
    
  case battery_level_3:
    save_buf[4] |= DISPLAY_BATTERY_LEVEL_3;
    break;	      
    
  default:
    save_buf[4] |= DISPLAY_BATTERY_OFF;
    break;
  }   
}


void led_display_update(void)
{	  
  u8 i;
  led_drv_send_cmd(0x02);  
  led_drv_send_cmd(0x44);      
  for(i=0; i<5; i++)
  {
    led_drv_send_cmd(0xc0+i*2);               
    led_drv_send_data(&save_buf[i], 1); 
    LED_CS_HIGH();
    delay_us(1);
  }  
  led_drv_send_cmd(0xc0);               
  led_drv_send_data(&save_buf[0], 1); 
  LED_CS_HIGH();
  delay_us(1);  
  led_drv_send_cmd(0x8c);
}

int dis_gpio_group[3]=
{
  AM_IO_LED_DIN  ,
  AM_IO_LED_CLK   ,
  AM_IO_LED_STB    
};

U8 init_dis_gpio(void)
{
  int ret = -1;
  for( int i=0; i<3; i++ )
  {
    ret = gpio_request_one(dis_gpio_group[i],GPIO_F_DIR_OUT);
    
    if(ret != -1)
    {
      ;
    }
    else
    {
      printf("LEDDRV io not ok\r\n");
    }
  }  
  return 0;
}
void led_drv_init(void)
{
  init_dis_gpio();
  LED_CS_HIGH();
  LED_CLK_HIGH();
  LED_DATA_HIGH();
  
  led_display_all_off();  
  led_display_num(0,0);
  led_display_num(1,1);
  led_display_num(2,2);
  led_display_num(3,3); 
  led_display_colon(1);    
  led_display_clean_mode(clean_mode_normal);
  led_display_battery(battery_level_3);
  led_display_update();
}

