#ifndef IRQ_SYSCALL_H
#define IRQ_SYSCALL_H
#include "stdint.h"

/****************************************************************
*Function   :  gpio_request
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统申请一个IO口
*CallBy     :  任何地方
*Input      :  gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  无
*Return     :  0：成功  ， -1:失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t (*io_request) (uint32_t gpio);

/****************************************************************
*Function   :  gpio_free
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统释放一个IO口
*CallBy     :  任何地方
*Input      :  gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_free) (uint32_t gpio); 

/****************************************************************
*Function   :  gpio_request_array
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统申请连续多个IO口
*CallBy     :  任何地方
*Input      :  参数
*              start: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              num  : 连续 IO口的个数
*              flags: am_gpio.h 里 GPIO_F_xxx的宏
*Output     :  无
*Return     :  0：成功  ， -1:失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t  (*io_request_array) (int32_t start, int32_t num, int32_t flags);

/****************************************************************
*Function   :  gpio_free_array
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统释放连续多个IO口
*CallBy     :  任何地方
*Input      :  参数
*              start: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              num  : 连续 IO口的个数
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_free_array)(int32_t start, int32_t num) ;

/****************************************************************
*Function   :  gpio_set_value
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  往GPIO输出一个电平
*CallBy     :  任何地方
*Input      :  参数
*              start: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              value: 0: 低电平输出    1： 高电平输出
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_set_value)(uint32_t gpio,int32_t value) ;

/****************************************************************
*Function   :  gpio_get_value
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取一个GPIO的输入电平
*CallBy     :  任何地方
*Input      :  参数
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  无
*Return     :  0： 低电平    1:高电平
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t (*io_get_value)(uint32_t gpio);

/****************************************************************
*Function   :  gpio_to_exti
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  把一个GPIO配置成外部中断
*CallBy     :  任何地方
*Input      :  参数
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*io_to_exti) (uint32_t gpio);

/****************************************************************
*Function   :  gpio_set_afio
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  把一个GPIO配置多功能复用模式
*CallBy     :  任何地方
*Input      :  参数
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              afio: 参考am_gpio.h  AFIO_MODE_Enum
*Output     :  无
*Return     :  0： 成功    -1：失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t  (*set_afio) (uint32_t gpio, uint32_t afio);

/****************************************************************
*Function   :  gpio_request_one
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统申请一个GPIO口，然后可以指定flags
*CallBy     :  任何地方
*Input      :  参数
*              gpio: GPIOX(x) X(A,B,C,D,E,F,G)  x(0~15) 
*              flags: 参考am_gpio.h 里 GPIO_F_xxx的宏
*Output     :  无
*Return     :  0： 成功    -1：失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int32_t  (*io_request_one) (uint32_t gpio, unsigned long flags) ;

/****************************************************************
*Function   :  remote_val
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  暂无使用
*CallBy     :  任何地方
*Input      :  参数
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*remote_val) (uint8_t index, uint8_t val);

/****************************************************************
*Function   :  get_adc
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取adc物理缓冲区的地址，adc总共有32个32bit的连续缓冲区
*CallBy     :  任何地方
*Input      :  无
*Output     :  无
*Return     :  adc物理通道缓冲区的首地址
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef u32* (*adc_val) (void);

/****************************************************************
*Function   :  robot_lt_update
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统注册light touch的结果信息
*CallBy     :  任何地方
*Input      :  参数
*              val: signal_result[]
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef void (*robot_lt_update_t) (uint8_t *val);

/****************************************************************
*Function   :  printk
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  打印函数
*CallBy     :  任何地方
*Input      :  参数
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*printf_t)(const char *fmt, ...);

/****************************************************************
*Function   :  write
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  往设备驱动中写入数据
*CallBy     :  任何地方
*Input      :  参数
*              fd : 设备句柄好
*              buf: 要写的数据 
*              len: 要写的数据的长度
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*write_t) (int fd, uint8_t *buf, uint16_t len);

/****************************************************************
*Function   :  read
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  往设备驱动中读数据
*CallBy     :  任何地方
*Input      :  参数
*              fd : 设备句柄好
*              buf: 存放读入的数据 
*              len: 要读入的数据的长度
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*read_t)  (int fd, uint8_t *buf, uint16_t len);

/****************************************************************
*Function   :  ioctl
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  设备控制函数
*CallBy     :  任何地方
*Input      :  参数
*              fd : 设备句柄好
*              cmd: 命令字，参考am_device.h 各个设备命令字
*              arg: 对应命令字的参数
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int  (*ioctl_t) (int fd, int cmd,void *arg);

/****************************************************************
*Function   :  runing_mode
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取系统的运行状态
*CallBy     :  任何地方
*Input      :  无
*Output     :  无
*Return     :  系统的状态,参考 ROBOT_STATE_E 
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef U8   (*runing_mode_t) (void);

typedef void (*robot_pos_get_t)(AM_Pose *out);
  
/****************************************************************
*Function   :  open
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  打开设备驱动
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              devName : 设备名称
*              devId   : 设备id 
*Output     :  无
*Return     :  0：成功    -1：失败
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int (*open_t)(int devName, int devId);


/****************************************************************
*Function   :  close
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  关闭设备
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              fd : 设备句柄好
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef int (*close_t)(int fd);

/****************************************************************
*Function   :  timer_ms
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取系统绝对时间，单位ms，从系统上电开始计算
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
typedef uint32_t (*timer_ms_t)(void);

/****************************************************************
*Function   :  timer_elapsed
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  计算系统逝去的时间
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              milli_start： 过去参考的时间
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef uint32_t (*timer_elapsed_t)(uint32_t milli_start);

/****************************************************************
*Function   :  get_microsecond
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取微秒的时间
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  绝对微秒数
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
typedef uint64_t (*get_microsecond_t)(void);
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
typedef uint16_t (*lock_irq_t)(void);

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
typedef uint16_t (*unlock_irq_t)(void);
typedef void (*register_ir_ops_t)(void* docking_ops);
typedef void (*get_dock_api_t)(void *api[], int api_size);
typedef struct
{
  io_request        gpio_request;
  io_free           gpio_free;
  io_request_array  gpio_request_array;
  io_free_array     gpio_free_array;
  io_set_value      gpio_set_value;
  io_get_value      gpio_get_value;
  io_to_exti        gpio_to_exti;
  set_afio          gpio_set_afio;
  io_request_one    gpio_request_one;
  remote_val        set_remote_val;
  adc_val           get_adc;
  robot_lt_update_t robot_lt_update;
  printf_t          printk; 
  write_t           write;
  read_t            read;
  ioctl_t           ioctl; 
  runing_mode_t     runing_mode;
  robot_pos_get_t   robot_pos_get;
  open_t            open;
  close_t           close;
  timer_ms_t        timer_ms;
  timer_elapsed_t   timer_elapsed;
  get_microsecond_t get_microsecond;
  lock_irq_t        lock_irq;
  unlock_irq_t      unlock_irq;
  register_ir_ops_t register_ir_ops;
  get_dock_api_t    get_dock_api;
}syscall_fun;

#endif