//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
/*edit by lyy*/

#include "ui-manager.h"
#include "am_app_cfg.h"
#include "am_device.h"
#include "local_process/local_process.h"
#include "syscall_api.h"
#include "am_uart_api.h"
#include "am_tm_api.h"
#include "am_spi_cfg.h"


extern void init_irq_syscall(void);
static u32 AppTaskStk[APP_TASK_STK_SIZE];

int main()
{
  /*first call*/
  init_irq_syscall();
  /*driver init*/
  sys_uart_cfg();
  sys_tm_cfg();
  sys_spi_cfg();
  printf("welcome to app\r\n");
  register_ui_config(get_ui_manager());
  local_app_init();
  pthread_create(main_app_task,&AppTaskStk[APP_TASK_STK_SIZE-1],APP_TASK_MAIN_PRIO,&AppTaskStk[0],APP_TASK_STK_SIZE);
  printf("app-init-ok!\r\n");
  return 0;
}
