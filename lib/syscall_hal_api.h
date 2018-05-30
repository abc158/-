#ifndef __SYSCALL_DEVICE_API_H__
#define __SYSCALL_DEVICE_API_H__
//#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"
//#include "irq_syscall.h"
//#include "hal_amicro_gpio.h"

extern syscall_fun *syscall_irq;

#define  printf             syscall_irq->printk
#define  gpio_request_one   syscall_irq->gpio_request_one
#define  gpio_request       syscall_irq->gpio_request
#define  gpio_free          syscall_irq->gpio_free
#define  gpio_request_array syscall_irq->gpio_request_array
#define  gpio_free_array    syscall_irq->gpio_free_array
#define  gpio_set_value     syscall_irq->gpio_set_value
#define  gpio_get_value     syscall_irq->gpio_get_value
#define  robot_lt_update    syscall_irq->robot_lt_update
#define  write              syscall_irq->write
#define  read               syscall_irq->read
#define  ioctl              syscall_irq->ioctl
#define  write              syscall_irq->write
#define  open               syscall_irq->open
#define  close              syscall_irq->close
#define  sys_runing_mode_get    syscall_irq->runing_mode
#define  robot_pos_get      syscall_irq->robot_pos_get
#define  timer_ms           syscall_irq->timer_ms
#define  timer_elapsed      syscall_irq->timer_elapsed
#define  get_microsecond    syscall_irq->get_microsecond
#define  lock_irq           syscall_irq->lock_irq
#define  unlock_irq         syscall_irq->unlock_irq
#define  register_ir_ops    syscall_irq->register_ir_ops
#define  get_dock_api       syscall_irq->get_dock_api

/*!!!!!!!!!!!!!!下面的函数不能在中断上下文使用!!!!!!!!!!!!*/
/****************************************************************
*Function   :  request_irq
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向设备请求中断
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              fd  : 设备句柄好
*              f   : 中断处理函数
*              arg : 中断处理函数的参数
*Output     :  无
*Return     :  0：成功   -1：失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int request_irq(int fd , long f, long arg);

#endif
