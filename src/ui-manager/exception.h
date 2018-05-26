#ifndef __UI_EXCEPTION__
#define __UI_EXCEPTION__

extern void sys_exception_monitor(void);
extern void exception_handle(void);
extern void set_ui_error_number(UI_ERROR_NUMBERS_E id);
extern UI_ERROR_NUMBERS_E get_ui_error_num(void);
extern void clear_ui_error_number(void);

#endif