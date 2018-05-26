//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "syscall_api.h"
#include "am_gpio.h"
#include "ui-song-player.h"



#ifndef SONG_PLAY_TIME_LIMIT
#define SONG_PLAY_TIME_LIMIT 5000
#endif

#ifdef USE_BEEP_PLAYER
#define BEEP_TURN_ON_TIME_COUNT 50
#define BEEP_SONG_INDEX_MAX  50

uint32_t beep_on_count = 0;
uint8_t beep_chopping_count[51] = {
  1/*00*/, 1/*01*/, 1/*02*/, 5/*03*/, 1/*04*/, 1/*05*/, 1/*06*/, 1/*07*/, 1/*08*/, 1/*09*/,
  1/*10*/, 1/*11*/, 1/*12*/, 1/*13*/, 1/*14*/, 1/*15*/, 1/*16*/, 1/*17*/, 10/*18*/, 10/*19*/,
  10/*20*/, 1/*21*/, 1/*22*/, 3/*23*/, 1/*24*/, 1/*25*/, 1/*26*/, 1/*27*/, 1/*28*/, 1/*29*/,
  1/*30*/, 1/*31*/, 5/*32*/, 5/*33*/, 5/*34*/, 5/*35*/, 5/*36*/, 5/*37*/, 5/*38*/, 5/*39*/,
  5/*40*/, 5/*41*/, 5/*42*/, 5/*43*/, 5/*44*/, 5/*45*/, 5/*46*/, 5/*47*/, 5/*48*/, 5/*49*/,
  5/*50*/
};

int songplayer_init(void)
{      
  beep_on_count = 0;
  gpio_set_value(HAL_IO_AUDIO_PWR_EN, 0);
  return 0;    
}


int songplayer_play_id(int id, int replace)
{
  if (id < BEEP_SONG_INDEX_MAX) {
    beep_on_count = (uint32_t)(BEEP_TURN_ON_TIME_COUNT*2*beep_chopping_count[id]);
  } else {
    beep_on_count = (uint32_t)(BEEP_TURN_ON_TIME_COUNT*2);
  }    
  gpio_set_value(HAL_IO_AUDIO_PWR_EN, 1);
  return 0;
}

int songplayer_routine(void)
{
    if (beep_on_count > 0)
    {
        beep_on_count--;
        if (beep_on_count == 0) {
            gpio_set_value(HAL_IO_AUDIO_PWR_EN, 0);
        } else {
            if ( beep_on_count % BEEP_TURN_ON_TIME_COUNT == 0) {
                if (beep_on_count/BEEP_TURN_ON_TIME_COUNT%2 == 0) {
                    gpio_set_value(HAL_IO_AUDIO_PWR_EN, 1);
                } else {
                    gpio_set_value(HAL_IO_AUDIO_PWR_EN, 0);
                }
            }
        }
    } else {
        gpio_set_value(HAL_IO_AUDIO_PWR_EN, 0);
    }
    return 0;
}

int songplayer_quit_playing(void)
{
    beep_on_count = 0;
    gpio_set_value(HAL_IO_AUDIO_PWR_EN, 0);
    return 0;
}

int songplayer_is_playing(void)
{
  return (beep_on_count > 0) ? 1 : 0;
}

#else

u8 songplayer_en = 1;

int songplayer_init(void)
{          
  return voiceplayer_init();    
}

int songplayer_quit_playing(void)
{
  return voiceplayer_quit_playing();
}

int songplayer_is_playing(void)
{
  return voiceplayer_is_playing();
}

int songplayer_play_id(int id, int replace)
{
  if (songplayer_en)
  {
    voiceplayer_play_index(id, replace);
  }
  return 0;
}

int songplayer_routine(void)
{
    return 0;
}

void set_songplayer_onoff(u8 onoff)
{
  songplayer_en = onoff;
}


u8 get_songplayer_onoff(void)
{
	return songplayer_en;
}

#endif


int songplayer_wait_for_song_completion(void)
{
  uint32_t start_time = timer_ms();
  uint32_t time_diff;
  uint32_t cur_time;
  
  while(songplayer_is_playing())
  {
    cur_time = timer_ms();
    time_diff = cur_time - start_time;
    if(time_diff > SONG_PLAY_TIME_LIMIT) {
      break;
    }
  }
  return 0;
}






