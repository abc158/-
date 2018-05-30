#ifndef __SYSCALL_SYS_API_H__
#define __SYSCALL_SYS_API_H__

#define EXPORT extern

typedef void (*TaskFunc)(void);
typedef void (*power_func_t)(uint8_t handler_time);


/*保留，目前暂时没有使用*/
typedef struct {
  power_func_t sleep;
  power_func_t wake;
}power_ctrl_t;

/*枚举类型说明： 系统信息类型定义*/
typedef enum
{
  SYS_SCHEDULE_INFO,/*系统预约信息*/
  SYS_TIME_INFO,    /*系统时间信息*/
  SYS_STATE_INFO,   /*系统状态信息*/
  SYS_SENSOR_INFO,  /*系统cliff，light，adc信息，目前可以直接通过adc get接口访问*/
  SYS_CURRENT_INFO,  /*系统电流信息，比如bat，电机， 获取到的值是去抖过的平均值， 目前也可以通过adc get接口访问瞬时值*/
  SYS_BITMAP_INFO   /*获取地图信息*/
}INFO_TYPE_E;


/*宏定义说明：  定时器中断注册类型*/
#define HZ_8K 4  /*注册的timer函数，每8K Hz就运行一次*/
#define HZ_4K 1  /*注册的timer函数，每4K Hz就运行一次*/
#define HZ_64 2  /*注册的timer函数，每64 Hz就运行一次*/
#define HZ_1K 3  /*注册的timer函数，每1K Hz就运行一次*/

/*!!!!!!!!!!!!!!下面的函数不能在中断上下文使用!!!!!!!!!!!!*/
/****************************************************************
*Function   :  act_ctrl_cmd
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统发起控制行为的命令
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              pkt_p: 是一个结构体命令，请参考am_date_base.h 里的struct command。
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void act_ctrl_cmd( uint8_t *pkt_p );

/****************************************************************
*Function   :  sys_info_get
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取系统信息
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              type: 请参考 INFO_TYPE_E 注释
*              addr: 这个就是存在获取回来的信息的结构体地址
                  struct sys_state_info
                  struct sys_current_info
                  struct ui_tm
                  struct sys_sensor_adc_info
                  struct schedule_time_info  
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void sys_info_get(INFO_TYPE_E type, long addr);



EXPORT void set_power_callback(power_ctrl_t *func);

/****************************************************************
*Function   :  register_ui_config
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统注册ui_manager 的信息
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              cfg ： ui_manager_t结构体，提供给SDK固件使用的函数
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void register_ui_config(const ui_manager_t *cfg);

/****************************************************************
*Function   :  sys_timer_register
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统注册 特许HZ 中断处理函数
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              time_hz ： HZ_8K，HZ_4K，HZ_1K，HZ_64, 
*              handler :  注册的中断处理函数
*              pri     :  优先级
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int8_t sys_timer_register(u8 time_hz, long handler, u8 pri);

/****************************************************************
*Function   :  timer_task_register
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向系统注册中断处理函数（最小单位是1ms）
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              task :  注册的中断处理函数
*              interval     :  周期间隔，比如10，就是10ms运行一次，如此类推
*              periodic     ： 0： 单次运行，  1：周期运行
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int timer_task_register( TaskFunc task, uint32_t interval, uint8_t periodic);//interval 1ms

/****************************************************************
*Function   :  timed_tasks_unregister
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  向卸载注册中断处理函数
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              task :  注册的中断处理函数
*Output     :  无
*Return     :  0： ok，  1: fail
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT int timed_tasks_unregister( TaskFunc task );
//systime 
/****************************************************************
*Function   :  get_clocktime_seconds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  系统运行时间：当前秒数
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 ： 秒(0~59)
*Others     :  系统睡眠时，这个时间会停止计时
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_seconds(void);

/****************************************************************
*Function   :  get_clocktime_minutes
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  系统运行时间：当前分
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 ： 分(0~59)
*Others     :  系统睡眠时，这个时间会停止计时
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_minutes(void);//系统起来后，逝去的分钟数

/****************************************************************
*Function   :  get_clocktime_hours
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  系统运行时间：当前时
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 ： 时(0~23)
*Others     :  系统睡眠时，这个时间会停止计时
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_hours(void);  //系统起来后，逝去的小时数

/****************************************************************
*Function   :  get_total_seconds
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  系统运行时间：累计的总秒数
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 ： 时(0~0xfffffff)
*Others     :  系统睡眠时，这个时间会停止计时
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT u32 get_total_seconds(void);    

/****************************************************************
*Function   :  get_clocktime_days
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  系统运行时间：累计的总天数
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 ： 时(0~0xffff)
*Others     :  系统睡眠时，这个时间会停止计时
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT s16 get_clocktime_days(void); 

/****************************************************************
*Function   :  get_sum_ticks
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  系统运行累计的总tick数
*CallBy     :  任何地方，中断上下文除外
*Input      :  无
*Output     :  无
*Return     :  返回 ： 时(0~0xfffffff)
*Others     :  系统睡眠时，这个时间会停止计数
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT u32 get_sum_ticks(void);

/****************************************************************
*Function   :  get_adc_chan_val
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取adc通道的adc值
*CallBy     :  任何地方，中断上下文除外
*Input      :  参数
*              adc_chan ： 参考 adc_chan.h ADC_ID_E
*              无
*Output     :  无
*Return     :  返回，adc值
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U16   get_adc_chan_val(ADC_ID_E adc_chan);

/****************************************************************
*Function   :  get_main_software_version
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  获取系统的版本信息
*CallBy     :  测试模式下
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回，版本号，比如50
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U32   get_main_software_version(void);

/****************************************************************
*Function   :  chipid_read
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  读取chipid 信息
*CallBy     :  测试模式下
*Input      :  参数
*              无
*Output     :  无
*Return     :  返回， chipid
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT u32   chipid_read(void);

/****************************************************************
*Function   :  vram_read
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  读取vram区数据,vram区最大可以读32 word。
*CallBy     :  
*Input      :  参数
*              offset：偏移地址,word位单位
*              data  ：数据
*              len   ：长度 ， word为单位
*Output     :  无
*Return     :  返回， 0
*Others     :  这个接口操作是word为单位，比如vram_read(2,buf,10), 意思就是从偏移第2 个word开始，读取10个word的数据。
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN vram_read(uint16_t offset , uint32_t *data,uint16_t len);

/****************************************************************
*Function   :  vram_write
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  写数据到vram区，ram区最大可以写32 word。
*CallBy     :  
*Input      :  参数
*              offset：偏移地址，word位单位，一个word为4byte
*              data  ：数据
*              len   ：长度，word为单位，一个word为4byte
*Output     :  无
*Return     :  返回， 0
*Others     :  这个接口操作是word为单位，比如vram_write(2,buf,10), 意思就是从偏移第2 个word开始，写入10个word的数据。
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN vram_write(uint16_t offset , uint32_t *data,uint16_t len);

/****************************************************************
*Function   :  wall_follow_is_running
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  是否处于沿边状态
*CallBy     :  
*Input      :  参数
               无
*Output     :  无
*Return     :  返回， 0：否   1：是
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN wall_follow_is_running(void);

/****************************************************************
*Function   :  forbit_sys_handle_rebound
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  禁止或者允许系统处理rebound流程（比如bump，cliff等等,参考Rebound 定义）
*CallBy     :  
*Input      :  参数
               rebound ： Rebound类型
                en      ： 1: forbit   0：allow
*Output     :  无
*Return     :  返回， 0：否   1：是
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void forbit_sys_handle_rebound(Rebound rebound, u8 en);


/****************************************************************
*Function   :  robot_xy_dist
*Author     :  lyy    
*Date       :  2017.6.28
*Description:  计算两个坐标点之间的直线距离
*CallBy     :  
*Input      :  参数
               AM_xy a： a点坐标 
               AM_xy b:  b点坐标
*Output     :  无
*Return     :  返回， 两点之间的距离
*Others     :  robot_pos_get获取当前点的坐标系
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT am_real robot_xy_dist(AM_xy *a, AM_xy *b);

/****************************************************************
*Function   :  motor_encode_get
*Author     :  lyy    
*Date       :  2017.6.28
*Description:  获取轮子码盘转动的tick数,这个可以用来测试码盘比。
*CallBy     :  
*Input      :  参数
*Output     :  无
*Return     :  返回， motor_encode_t 类型结构体
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT motor_encode_t *motor_encode_get(void);

/****************************************************************
*Function   :  system_reboot
*Author     :  lyy    
*Date       :  2017.7.7
*Description:  系统复位。
*CallBy     :  
*Input      :  参数
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.7.7       v1.0         build this function
******************************************************************/
EXPORT void system_reboot(void);

/****************************************************************
*Function   :  update_dirt_level
*Author     :  lyy    
*Date       :  2017.7.25
*Description:  更新垃圾检测的数量等级
*CallBy     :  
*Input      :  0： 无垃圾 1：数量少  2：数量多
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.7.25       v1.0         build this function
******************************************************************/
EXPORT void update_dirt_level(u8 level);

/****************************************************************
*Function   :  local_map_reset
*Author     :  lyy    
*Date       :  2017.8.231
*Description:  复位地图信息
*CallBy     :  
*Input      :  resume = 0， 其他值保留
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.8.31       v1.0         build this function
******************************************************************/
EXPORT int local_map_reset(int resume);

/****************************************************************
*Function   :  rate_gyro_init
*Author     :  lyy    
*Date       :  2017.8.231
*Description:  陀螺仪初始化
*CallBy     :  
*Input      :  resume = 0， 其他值保留
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.8.31       v1.0         build this function
******************************************************************/
EXPORT void rate_gyro_init(void);

/****************************************************************
*Function   :  set_robot_default_accel
*Author     :  lyy    
*Date       :  2017.10.26
*Description:  设置默认的加速度
*CallBy     :  
*Input      : accel： >0
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.10.26       v1.0         build this function
******************************************************************/
EXPORT void set_robot_default_accel(int accel);

/****************************************************************
*Function   :  afio_exit
*Author     :  lyy    
*Date       :  2018.1.3
*Description:  释放afio
*CallBy     :  
*Input      :  afio_t
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.8.31       v1.0         build this function
******************************************************************/
EXPORT unsigned char afio_exit(afio_t *);

/****************************************************************
*Function   :  afio_cfg
*Author     :  lyy    
*Date       :  2018.1.3
*Description:  配置afio
*CallBy     :  
*Input      :  afio_t
*Output     :  无
*Return     :  无
*Others     :  
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    lyy            17.10.26       v1.0         build this function
******************************************************************/
EXPORT unsigned char afio_cfg(afio_t *);

EXPORT void gpio_init(int gpioNum, int dir,int pr_mode);

EXPORT int check_other_phread_is_normal(void);

#endif
