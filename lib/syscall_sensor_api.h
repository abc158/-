#ifndef __SYSCALL_SENSOR_API_H__
#define __SYSCALL_SENSOR_API_H__
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

/****************************************************************
*Function   :  robot_set_going_speed
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设置清扫的速度（单位mm/s）
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              speed :  要设置的速度
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_going_speed(uint16_t speed);

/****************************************************************
*Function   :  robot_set_adc_adjust
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设置adc的校准值
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              val :  校准值
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_adc_adjust(uint16_t val);

/****************************************************************
*Function   :  robot_set_action_funs
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  动态配置各路电机的是否开启的功能
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              funs :  参考 结构体 clean_motor_config_t
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_action_funs(uint16_t funs);

/****************************************************************
*Function   :  set_motor_vels
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  驱动轮子行走
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              left :   左轮的速度  ， =0， 停止； <0 后退  >0，前进  单位是mm/s
*              right :  右轮的速度  ， =0， 停止； <0 后退  >0，前进  单位是mm/s
*              acc   ：  加速度 ，  单位mm/s
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_motor_vels( int16_t left,int16_t right, uint16_t acc);

/****************************************************************
*Function   :  get_motor_speeds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取轮子实际速度
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              left :   左轮的速度  单位是ticks/s
*              right :  右轮的速度  单位是ticks/s
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void get_motor_speeds( int16_t *left,int16_t *right);

/****************************************************************
*Function   :  get_commanded_speeds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取轮子预设的速度
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              left :   左轮的速度  单位是ticks/s
*              right :  右轮的速度  单位是ticks/s
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void get_commanded_speeds( int16_t *left,int16_t *right);

/****************************************************************
*Function   :  get_cliff_state
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取cliff的状态
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 CliffState类型
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT CliffState get_cliff_state(void);

/****************************************************************
*Function   :  get_bump_state
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取bump的状态
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 BumpState 类型
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BumpState  get_bump_state(void);

/****************************************************************
*Function   :  set_slip_high_throd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  恢复slip 阈值为正常值
*CallBy     :  任何地方，中断上下文除外
*Input      :  val： 阈值 ，越高slip检测越不灵敏。
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_slip_normal(void);

/****************************************************************
*Function   :  set_slip_high_throd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设置slip打滑判断的阈值
*CallBy     :  任何地方，中断上下文除外
*Input      :  val： 阈值 ，越高slip检测越不灵敏。
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_slip_high_throd(float val);

/****************************************************************
*Function   :  set_stasis_normal
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  恢复stasis 阈值为正常值，检测灵敏度高
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_stasis_normal(void);

/****************************************************************
*Function   :  set_stasis_high_throd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设置stasis 阈值为高，则检测灵敏度低
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void set_stasis_high_throd(void);

/****************************************************************
*Function   :  get_gyro_angle
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取陀螺仪角度信息
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回就是角度度数信息，如-90,90
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int  get_gyro_angle(void);

/****************************************************************
*Function   :  get_gyro_radian
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取陀螺仪弧度信息
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回就是弧度信息，如-PI,PI
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT float get_gyro_radian(void);

/****************************************************************
*Function   :  set_charging_enable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  是否使能充电功能
*CallBy     :  任何地方，中断上下文除外
*Input      :  on： 1：使能   0：不使能
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_charging_enable(U16 on);

/****************************************************************
*Function   :  v_ref_init
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  充电初始化
*CallBy     :  开始使能充电前调用
*Input      :  无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  v_ref_init(void);

/****************************************************************
*Function   :  set_charge_current
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取当前实际的充电电流 (单位：mA)
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*Output     :  无
*Return     :  返回 实际的充电电流值 (mA)
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_charge_current(u16 val);

/****************************************************************
*Function   :  set_adaptor_output_voltage
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设置充电适配器的电压值 (单位：V)
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              val：  比如22V， 19V，24V
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_adaptor_output_voltage(u16 val);

/****************************************************************
*Function   :  set_charge_voltage
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设置充电的电压值 (单位：adc值)
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              adc_val:  电池充满后对于伏的adc值，比如目前16.8V 对应时3628
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void  set_charge_voltage(u16 adc_val);

/****************************************************************
*Function   :  charge_maybe_full
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  电池充满的判断（可能充满）
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              无
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U8    charge_maybe_full(void);
/****************************************************************
*Function   :  gyro_is_ok
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  判断陀螺仪是否可用
*CallBy     :  测试模式下
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回， 1：ok，  0：err
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U8    gyro_is_ok(void);

/****************************************************************
*Function   :  robot_set_slowdown_div
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设置减速分频比（lt 遇到前面障碍物时减速比，基于当前速度*div）
*CallBy     :  
*Input      :  参数
*              div : 分频比
*              无
*Output     :  无
*Return     :  返回， chipid
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void robot_set_slowdown_div(float div);

#endif
