#ifndef __SYSCALL_THREAD_API_H__
#define __SYSCALL_THREAD_API_H__
//#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"
//#include "ui-manager.h"
//#include "irq_syscall.h"
//#include "hal_amicro_gpio.h"
//#include "afio.h"
//#include "am_robot_type.h"
//#include "docking-core.h"
//#include "adc_chan.h"
//#include "ui-test_cmd_handle.h"
//#include "am_rebound.h"

#define EXPORT extern

/*!!!!!!!!!!!!!!下面的函数不能在中断上下文使用!!!!!!!!!!!!*/

/****************************************************************
*Function   :  lock_irq
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  锁中断
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  一般不建议随便调用，因为会影响系统整体性能
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/ 
EXPORT uint16_t lock_irq(void);

/****************************************************************
*Function   :  unlock_irq
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  解除锁中断
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  一般不建议随便调用，因为会影响系统整体性能
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/ 
EXPORT uint16_t unlock_irq(void);

/****************************************************************
*Function   :  pthread_create
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  创建线程
*CallBy     :  任何地方，除了中断上下文外
*Input      :  参数
*             task：    线程函数
*             p_arg:    线程函数是入口参数
*             ptos :    线程栈顶
*             prio ：   线程优先级
*             pbos :    线程栈底
*             stk_size: 线程栈深度
*Output     :  无
*Return     :  0： 成功，  非0： 失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_create (void (*task)(void *p_arg),u32 *ptos,u8  prio,u32  *pbos, u32 stk_size);

/****************************************************************
*Function   :  pthread_delete
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  强制销毁线程
*CallBy     :  任何地方，除了中断上下文外
*Input      :  参数
*             prio ：   线程优先级
*Output     :  无
*Return     :  0： 成功，  非0： 失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_delete(uint8_t prio);

/****************************************************************
*Function   :  pthread_change_prio
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  改变线程的优先级
*CallBy     :  任何地方，除了中断上下文外
*Input      :  参数
*             oldprio ：   线程当前的优先级
*             newprio ：   线程新的优先级
*Output     :  无
*Return     :  0： 成功，  非0： 失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_change_prio (uint8_t  oldprio,uint8_t  newprio);

/****************************************************************
*Function   :  pthread_resume
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  线程恢复
*CallBy     :  任何地方，除了中断上下文外
*Input      :  参数
*             prio ：   线程优先级
*Output     :  无
*Return     :  0： 成功，  非0： 失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_resume(uint8_t prio);

/****************************************************************
*Function   :  pthread_suspend
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  线程挂起
*CallBy     :  任何地方，除了中断上下文外
*Input      :  参数
*             prio ：   线程优先级
*Output     :  无
*Return     :  0： 成功，  非0： 失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_suspend(uint8_t prio);

/****************************************************************
*Function   :  sleep
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  线程睡眠
*CallBy     :  任何地方，除了中断上下文外
*Input      :  参数
*             tick ：   线程睡眠的tick数，一个tick 10ms
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void sleep (uint32_t ticks);

/****************************************************************
*Function   :  pthread_wakeup
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  线程睡眠唤醒（如果一个线程sleep了，可以用这个唤醒）
*CallBy     :  任何地方，除了中断上下文外
*Input      :  参数
*             prio ：   线程优先级
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t pthread_wakeup (uint8_t prio) ;

/****************************************************************
*Function   :  pthread_sched_lock
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  锁住线程调度
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void pthread_sched_lock(void);

/****************************************************************
*Function   :  pthread_sched_unlock
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  打开线程调度
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void pthread_sched_unlock(void);

#endif
