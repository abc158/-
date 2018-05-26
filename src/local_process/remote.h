#ifndef __REMOTE__
#define __REMOTE__

#define TOTAL_IR_NUM 15
#define BUFFER_NUM   4

extern U8 remote_get_val(void);
extern U8 is_empty_buffer(void);
extern void remote_set_val(U8 val);
static void parseRemote( U16 remoteMatrix );
static void handle_remote( void );
extern void remote_routine(void);
extern void _remote_exit(void);
extern void _remote_init(void);
extern ir_state_t *get_ir_state(void);
extern void ir_rx_decode(void);

#endif
