#ifndef __ACT__
#define __ACT__

typedef enum
{
	UI_IDLE=0x1,
    UI_CLOCK,
    UI_SCHEDULE,    
    UI_WAIT_SLEEPING,
    UI_ENTER_SLEEPING,
    UI_WAKE_UP,
    UI_TEST,
    UI_ERROR,
    UI_FULLGO
}UI_STATE_E;





extern void act_routine(void);
extern void _act_init(void);
extern void _act_exit(void);
extern UI_STATE_E get_ui_state(void);
extern void set_ui_state(UI_STATE_E s);
extern void ui_handle_idle(void);
extern U8 get_enter_sleep_state(void);
extern void set_enter_sleep_state(U8 state);
extern void set_start_docking_state(u8 state);
extern u8 get_start_docking_state(void);
extern void set_fullgo_state(u8 state);
extern u8 get_fullgo_state(void);
void state_save_and_recover_grid(u8 flag);
void state_recover(void);
u8 get_robot_current_state(void);
extern u8 get_save_state(void);
extern void set_save_state(u8 s);
#endif
