#ifndef _SONG_PLAYER_H_
#define _SONG_PLAYER_H_

#include "ui-typedef.h"

//#define USE_BEEP_PLAYER

//audio file index in spi flash
typedef enum 
{
    VOICE_ID_START_CHARGE       = 1,
    VOICE_ID_BE_PICKED_UP       = 2,
    VOICE_ID_ERROR_NOTIFY       = 3,
    VOICE_ID_START_MOP          = 4,
    VOICE_ID_FINISH_CLEANING    = 5,
    VOICE_ID_BUTTON_CLICK       = 6,    
    VOICE_ID_BUTTON_POWER_ON    = 7,    
    VOICE_ID_CHARGER_POWER_ON   = 8,    
    VOICE_ID_POWER_OFF          = 9,    
    VOICE_ID_GO_SLEEP           = 10,    
    VOICE_ID_WAKE_UP            = 11,
    VOICE_ID_BUTTON_REJECT      = 12,
    VOICE_ID_START_CLEANNING    = 13,
    VOICE_ID_REMOTE_CONTROL     = 14,
    VOICE_ID_RESET_CLEAN        = 15,
    VOICE_ID_NOT_READY_TO_CLEAN = 16,
    VOICE_ID_STOP_CLEAN         = 17,
    VOICE_ID_RESERVE_POWER      = 18,
    VOICE_ID_CRITICAL_LOW_POWER = 19,
    VOICE_ID_LOW_POWER          = 20,    
    VOICE_ID_CHARGER_BAD        = 21,
    VOICE_ID_ENTER_PAUSED       = 22,
    VOICE_ID_CONFIG_BACKING_UP  = 23,
    VOICE_ID_IS_DOCK_FOUND         = 24,
    VOICE_ID_READY_TO_CLEAN     = 25,
    VOICE_ID_JOB_COMPLETE       = 26,
    VOICE_ID_GOING_HOME         = 27,
    VOICE_ID_ERROR_STOP         = 28,
	VOICE_ID_GOING_DOCK         = 29,
	VOICE_ID_IS_DOCKED          = 30,
	VOICE_ID_STOP_DOCKING       = 31,
    VOICE_ID_ERROR_00_REVERSE               = 32,
    VOICE_ID_ERROR_01_LEFT_WHEEL_HANGED     = 33,
    VOICE_ID_ERROR_02_MAIN_BRUSH_ENTWINED   = 34,
    VOICE_ID_ERROR_03_RIGHT_WHEEL_HANGED    = 35,
    VOICE_ID_ERROR_04_LEFT_WHEEL_ENTWINED   = 36,
    VOICE_ID_ERROR_05_RIGHT_WHEEL_ENTWINED  = 37,
    VOICE_ID_ERROR_06_ALWAYS_CLIFFED        = 38,
    VOICE_ID_ERROR_07_SIDE_BRUSH_ENTWINED   = 39,
    VOICE_ID_ERROR_08_CODEPANEL_ERROR       = 40,
    VOICE_ID_ERROR_09_BUMP_ERROR            = 41,
    VOICE_ID_ERROR_10_REVERSE               = 42,
    VOICE_ID_ERROR_11_REVERSE               = 43,
    VOICE_ID_ERROR_12_CLIFF_ERROR           = 44,
    VOICE_ID_ERROR_13_REVERSE               = 45,
    VOICE_ID_ERROR_14_REVERSE               = 46,
    VOICE_ID_ERROR_15_VACCUM_MOTO_ENTWINED  = 47,
    VOICE_ID_ERROR_16_BIG_CURRENT           = 48,
    VOICE_ID_ERROR_17_SWITCH_OFF_CHARGING   = 49,
    VOICE_ID_ERROR_18_REVERSE               = 50, 
    VOICE_ID_ERROR_19_REVERSE               = 51,
    VOICE_ID_ERROR_20_REVERSE               = 52,
    VOICE_ID_ERROR_21_REVERSE               = 53,
    VOICE_ID_ERROR_22_REVERSE               = 54,
    VOICE_ID_ERROR_23_REVERSE               = 55,
    VOICE_ID_ERROR_24_REVERSE               = 56,
    VOICE_ID_ERROR_25_REVERSE               = 57,
    VOICE_ID_ERROR_26_REVERSE               = 58,
    VOICE_ID_ERROR_27_REVERSE               = 59,
    VOICE_ID_ERROR_28_REVERSE               = 60,
    VOICE_ID_ERROR_29_REVERSE               = 61,
    VOICE_ID_START_WALLFOLLOW               = 62,
    VOICE_ID_EX_TIMER_SETTING               = 63,
    VOICE_ID_EX_SCHEDULE_SETTING            = 64,
    VOICE_ID_EX_LOCAL_CLEANING              = 65,  
    VOICE_ID_EX_ENTER_TEST                  = 66,
    VOICE_ID_EX_SCHEDULE_CLEAR              = 67,
    VOICE_ID_EX_KEYSTONE_CLEANING           = 68,
    VOICE_ID_EX_07              = 69,
    VOICE_ID_EX_08              = 70,
    VOICE_ID_EX_09              = 71,
    VOICE_ID_EX_10              = 72,
    VOICE_ID_EX_11              = 73,
    VOICE_ID_EX_12              = 74,
    VOICE_ID_EX_13              = 75,
    VOICE_ID_EX_14              = 76,
    VOICE_ID_EX_15              = 77,
    VOICE_ID_EX_16              = 78,
    VOICE_ID_EX_17              = 79,
    VOICE_ID_EX_18              = 80,
    VOICE_ID_NUMBER
} AM_VOICE_INDEX_E;



#define SONG_ID_LOW_POWER                   VOICE_ID_LOW_POWER
#define SONG_ID_RESERVE_POWER               VOICE_ID_RESERVE_POWER
#define SONG_ID_CRITICAL_LOW_POWER          VOICE_ID_CRITICAL_LOW_POWER
#define SONG_ID_BUTTON_CLICK                VOICE_ID_BUTTON_CLICK
#define SONG_ID_BUTTON_REJECT               VOICE_ID_BUTTON_REJECT
#define SONG_ID_POWER_OFF                   VOICE_ID_POWER_OFF
#define SONG_ID_POWER_UP                    VOICE_ID_BUTTON_POWER_ON
#define SONG_ID_CHARGING_START              VOICE_ID_START_CHARGE
//#define SONG_ID_CHARGING_STOP,
#define SONG_ID_CHARGING_ERROR_STOP         VOICE_ID_CHARGER_BAD
//#define SONG_ID_CHARGING_COMPLETE,
#define SONG_ID_DOCKING_START               VOICE_ID_GOING_DOCK
#define SONG_ID_DOCKING_STOP                VOICE_ID_STOP_DOCKING
#define SONG_ID_DOCKING_COMPLETE            VOICE_ID_IS_DOCKED
#define SONG_ID_HOMING_START                VOICE_ID_GOING_HOME
#define SONG_ID_HOMING_STOP                 VOICE_ID_STOP_CLEAN
#define SONG_ID_HOMING_COMPLETE             VOICE_ID_FINISH_CLEANING
#define SONG_ID_NORMAL_CLEANING_START       VOICE_ID_START_CLEANNING
#define SONG_ID_NORMAL_CLEANING_STOP        VOICE_ID_STOP_CLEAN
#define SONG_ID_NORMAL_CLEANING_COMPLETE    VOICE_ID_JOB_COMPLETE
#define SONG_ID_SPOT_CLEANING_START         VOICE_ID_EX_LOCAL_CLEANING
#define SONG_ID_SPOT_CLEANING_STOP          VOICE_ID_STOP_CLEAN
#define SONG_ID_SPOT_CLEANING_COMPLETE      VOICE_ID_JOB_COMPLETE
#define SONG_ID_WALLFOLLOW_CLEANING_START   VOICE_ID_START_WALLFOLLOW
#define SONG_ID_WALLFOLLOW_CLEANING_STOP    VOICE_ID_STOP_CLEAN
#define SONG_ID_WALLFOLLOW_CLEANING_COMPLETE    VOICE_ID_JOB_COMPLETE
#define SONG_ID_BE_PICKED_UP VOICE_ID_BE_PICKED_UP
#define SONG_ID_START_MOP                    VOICE_ID_START_MOP
#define SONG_ID_BUTTON_CANCEL               VOICE_ID_ERROR_00_REVERSE
#define SONG_ID_WIFI_CONNET_OK              VOICE_ID_ERROR_00_REVERSE
#define SONG_ID_ENTER_PAUSED                VOICE_ID_ENTER_PAUSED
#define SONG_ID_ENTER_TEST                  VOICE_ID_EX_ENTER_TEST
#define SONG_ID_KEYSTONE_CLEANING           VOICE_ID_EX_KEYSTONE_CLEANING
int songplayer_init(void);
int songplayer_quit_playing(void);
int songplayer_is_playing(void);
int songplayer_play_id(int id, int replace);
int songplayer_wait_for_song_completion(void);
int songplayer_routine(void);
void set_songplayer_onoff(u8 onoff);
u8 get_songplayer_onoff(void);

#endif /* _SONG_PLAYER_H_ */

