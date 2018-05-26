#ifndef __SYSCALL_SONG_API_H__
#define __SYSCALL_SONG_API_H__
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
*Function   :  voiceplayer_init
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  初始化语音播放器
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  0：成功 -1：失败 
*Others     :  一般建议在系统初始化时调用
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_init(void);

/****************************************************************
*Function   :  voiceplayer_play_index
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  使用语音播放器播放语音条目
*CallBy     :  任何地方，除了中断上下文外
*Input      :  
*             index:             语音条目序号
*             force_replace :    是否替换当前正在播放的语音
*Output     :  无
*Return     :  0：成功 -1：失败 
*Others     :  一般建议在语音播放器空闲状态下调用，以免语音不完整
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_play_index(uint16_t index, uint8_t force_replace);

/****************************************************************
*Function   :  voiceplayer_play_stream
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  使用语音播放器播放语音条目
*CallBy     :  任何地方，除了中断上下文外
*Input      :  
*             stream:   语音数据在ROM空间的地址
*             size :    语音数据的大小（以byte为单位）
*Output     :  无
*Return     :  0：成功 -1：失败 
*Others     :  一般建议在语音播放器空闲状态下调用，以免语音不完整。
*              该播放模式默认优先级较高，会替换当前正在播放的语音。
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.7.25       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_play_stream(const uint8_t* stream, uint32_t size);

/****************************************************************
*Function   :  voiceplayer_quit_playing
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  停止播放语音
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  0：成功 -1：失败 
*Others     :  一般建议在语音播放器正播放语音时调用
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_quit_playing(void);

/****************************************************************
*Function   :  voiceplayer_is_playing
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  查看当前是否正在播放语音
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  1：正在播放语音 0：当前语音播放器处于空闲状态
*Others     :  无
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_is_playing(void);

/****************************************************************
*Function   :  voiceplayer_get_file_version
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  查看当前语音文件的版本号
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  任意值，与制作语音文件时工具输入一致
*Others     :  一般建议在语音播放器初如化之后调用
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_get_file_version(void);

/****************************************************************
*Function   :  voiceplayer_get_file_count
*Author     :  luenwan    
*Date       :  2017.4.20
*Description:  查看当前语音条目的个数
*CallBy     :  任何地方，除了中断上下文外
*Input      :  无
*Output     :  无
*Return     :  正常值大于0
*Others     :  一般建议在语音播放器初如化之后调用
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.6.9       v1.0         build this function
******************************************************************/ 
EXPORT int voiceplayer_get_file_count(void);

#endif
