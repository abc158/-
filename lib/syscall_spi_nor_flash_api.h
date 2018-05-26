#ifndef __SYSCALL_SPI_NOR_FLASH_API_H__
#define __SYSCALL_SPI_NOR_FLASH_API_H__
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
*Function   :  spi_nor_flash_read
*Author     :  luenwan    
*Date       :  2017.7.5
*Description:  读取nor flash里的数据
*CallBy     :  任何地方，除了中断上下文外
*Input      :  addr：开始地址(低24位有效) buf：数据缓冲地址 len: 读取数据的长度
*Output     :  无
*Return     :  > 0：成功 <= 0：失败 
*Others     :  无
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.7.5       v1.0         build this function
******************************************************************/ 
EXPORT int spi_nor_flash_read(uint32_t addr, uint8_t* buf, uint32_t len);

/****************************************************************
*Function   :  spi_nor_flash_write
*Author     :  luenwan    
*Date       :  2017.7.5
*Description:  将数据写入nor flash
*CallBy     :  任何地方，除了中断上下文外
*Input      :  addr：开始目标地址(低24位有效) buf：数据存储地址 len: 数据的长度
*Output     :  无
*Return     :  > 0：成功 <= 0：失败 
*Others     :  无
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.7.5       v1.0         build this function
******************************************************************/ 
EXPORT int spi_nor_flash_write(uint32_t addr, uint8_t* buf, uint32_t len);

/****************************************************************
*Function   :  spi_nor_flash_erase
*Author     :  luenwan    
*Date       :  2017.7.5
*Description:  擦除norflash里一段区间的内容。
*CallBy     :  任何地方，除了中断上下文外
*Input      :  addr：开始目标地址(低24位有效) len: 数据的长度
*Output     :  无
*Return     :  >= 0：成功 < 0：失败 
*Others     :  擦除最小单位为4096，addr,len必须为4096的整数倍(非4096整数倍情况下可能会擦除前后两个4096byte空间)
*History    : //修改历史
    <author>       <time>      <version>           <desc>
    luenwan            17.7.5       v1.0         build this function
******************************************************************/ 
EXPORT int spi_nor_flash_erase(uint32_t addr, uint32_t len);


#endif
