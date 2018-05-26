#ifndef __LIB_H__
#define __LIB_H__

#include "am_date_base.h"
#include "ui-commands.h"

extern void act_command_q(U32 command, U8 cmd_state, void *msg, U16 msg_len);
extern tm *get_current_time(void);
extern schedule_time_info *get_schedule_info(void);
extern schedule_time_info *get_tmp_schedule_info(void);
extern void make_schedule_time(tm *,bool);
extern void set_schedule_time(void);
extern void set_current_time(tm *t);
extern U32 send_schedule_cmd(void);
extern U32 send_time_cmd(void);
extern U32 send_standby_cmd(void);

extern U32 send_sleep_time_cmd(void);
extern void send_forward_cmd(CMD_STATE_E mode);
extern void send_backward_cmd(CMD_STATE_E mode);
extern void send_left_cmd(CMD_STATE_E mode);
extern void send_right_cmd(CMD_STATE_E mode);
extern void clean_out_key_flg(void);
#endif