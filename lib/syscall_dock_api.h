#ifndef __SYSCALL_DOCK_API_H__
#define __SYSCALL_DOCK_API_H__
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
*Function   :  dock_is_enable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  判断dock 功能是否开启了
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t dock_is_enable(void);

/****************************************************************
*Function   :  dock_core_enable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  启动 CORE 层.
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void dock_core_enable(void);


/****************************************************************
*Function   :  dock_core_disable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  关闭 CORE 层.
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void dock_core_disable(void);

/****************************************************************
*Function   :  unregister_dock_function
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  注消dock behavior
*CallBy     :  任何地方，中断上下文除外
*Input      :  dock behavior的优先级.
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void unregister_dock_function(U8 priorty);

/****************************************************************
*Function   :  register_dock_function
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  注册dock behavior
*CallBy     :  任何地方，中断上下文除外
*Input      :  Dock_Data数据结构指针.
*Output     :  无
*Return     :  TRUE：成功.  FLASE：失败.
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 register_dock_function(Dock_Data *dock_funtion);

/****************************************************************
*Function   :  register_dock_signals
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  注册红外信号检测函数
*CallBy     :  任何地方，中断上下文除外
*Input      :  获取红外信号函数指针
*Output     :  无
*Return     :  TRUE：成功.  FLASE：失败.
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 register_dock_signals(get_dock_signals dock_signals);


/****************************************************************
*Function   :  register_random_conut
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  注册随机统计函数.
*CallBy     :  任何地方，中断上下文除外
*Input      :  dock_random 指针，指向对应的随机统计函数
*Output     :  无
*Return     :  TRUE：成功.  FLASE：失败.
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 register_random_conut(get_random_count dock_random);

/****************************************************************
*Function   :  last_dock_behavior
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取上一次行为优先级
*CallBy     :  任何地方，中断上下文除外
*Input      :  无.
*Output     :  无
*Return     :  上一次行为优先级
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 last_dock_behavior(void);

/****************************************************************
*Function   :  current_dock_behavior
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取当前行为优先级
*CallBy     :  任何地方，中断上下文除外
*Input      :  无.
*Output     :  无
*Return     :  当前行为优先级
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 current_dock_behavior(void);

/****************************************************************
*Function   :  register_debouncer
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  注册 debouncer.
*CallBy     :  任何地方，中断上下文除外
*Input      :  debouncer_function 指针，指向对应的 debouncer
*Output     :  无
*Return     :  TRUE：成功.  FLASE：失败.
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN register_debouncer(Debouncer_Data *debouncer_funtion);

/****************************************************************
*Function   :  unregister_debouncer
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  注销 debouncer.
*CallBy     :  任何地方，中断上下文除外
*Input      :  debouncer_function 指针，指向对应的 debouncer
*Output     :  无
*Return     :  TRUE：成功.  FLASE：失败.
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN unregister_debouncer(Debouncer_Data *debouncer_funtion);

/****************************************************************
*Function   :  clear_debouncer
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  清除 debouncer 历史数据.
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void clear_debouncer(void);

/****************************************************************
*Function   :  get_random
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取随机角度.
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  随机角度，单位度
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U16 get_random(void);

/**************************************************
*功能： 控制旋转函数.
*参数： first，是否设置起始角度（起点），如果设置起始点，则会把当前角度设为0度，
*       rotation_angle，需要旋转的度数，单位度
*       l_speed，左轮速度
*       r_speed，右轮速度
*       care_bump，是否考虑 bump 或 cliff
*       result，是否转到目标角度
*               0，成功
*               1， 发生 cliff
*               2， 发生 bump
*返回值：旋转角度
*
**************************************************/
EXPORT S16 robot_turn(BOOLEAN first, S16 rotation_angle, S16 l_speed,S16 r_speed, BOOLEAN care_bump, S8 *result);

/**************************************************
*功能： 控制直行函数.
*参数： first，是否设置起点
*       distance，需要直行的距离， 单位毫米
*       speed，速度
*       care_bump，是否考虑 bump 或 cliff
*       result，是否转到达目的地
*               0，成功
*               1， 发生 cliff
*               2， 发生 bump
*返回值：直行了多少距离
*
**************************************************/
EXPORT S16 robot_drive_go(BOOLEAN first, S16 distance, S16 speed, BOOLEAN care_bump, S8 *result);

/**************************************************
*功能： 设置导航回座时，必须到达的位置点的个数
*参数： num:必须到达的位置点的个数，如果不设置，默认为1个
*返回值：无
**************************************************/
EXPORT void set_navi_no_docksignal(int num);
#endif
