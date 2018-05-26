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

/*!!!!!!!!!!!!!!����ĺ����������ж�������ʹ��!!!!!!!!!!!!*/
/****************************************************************
*Function   :  dock_is_enable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ж�dock �����Ƿ�����
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT uint8_t dock_is_enable(void);

/****************************************************************
*Function   :  dock_core_enable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ���� CORE ��.
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void dock_core_enable(void);


/****************************************************************
*Function   :  dock_core_disable
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  �ر� CORE ��.
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void dock_core_disable(void);

/****************************************************************
*Function   :  unregister_dock_function
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ע��dock behavior
*CallBy     :  �κεط����ж������ĳ���
*Input      :  dock behavior�����ȼ�.
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void unregister_dock_function(U8 priorty);

/****************************************************************
*Function   :  register_dock_function
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ע��dock behavior
*CallBy     :  �κεط����ж������ĳ���
*Input      :  Dock_Data���ݽṹָ��.
*Output     :  ��
*Return     :  TRUE���ɹ�.  FLASE��ʧ��.
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 register_dock_function(Dock_Data *dock_funtion);

/****************************************************************
*Function   :  register_dock_signals
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ע������źż�⺯��
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��ȡ�����źź���ָ��
*Output     :  ��
*Return     :  TRUE���ɹ�.  FLASE��ʧ��.
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 register_dock_signals(get_dock_signals dock_signals);


/****************************************************************
*Function   :  register_random_conut
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ע�����ͳ�ƺ���.
*CallBy     :  �κεط����ж������ĳ���
*Input      :  dock_random ָ�룬ָ���Ӧ�����ͳ�ƺ���
*Output     :  ��
*Return     :  TRUE���ɹ�.  FLASE��ʧ��.
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 register_random_conut(get_random_count dock_random);

/****************************************************************
*Function   :  last_dock_behavior
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ��һ����Ϊ���ȼ�
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��.
*Output     :  ��
*Return     :  ��һ����Ϊ���ȼ�
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 last_dock_behavior(void);

/****************************************************************
*Function   :  current_dock_behavior
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ��ǰ��Ϊ���ȼ�
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��.
*Output     :  ��
*Return     :  ��ǰ��Ϊ���ȼ�
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT S8 current_dock_behavior(void);

/****************************************************************
*Function   :  register_debouncer
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ע�� debouncer.
*CallBy     :  �κεط����ж������ĳ���
*Input      :  debouncer_function ָ�룬ָ���Ӧ�� debouncer
*Output     :  ��
*Return     :  TRUE���ɹ�.  FLASE��ʧ��.
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN register_debouncer(Debouncer_Data *debouncer_funtion);

/****************************************************************
*Function   :  unregister_debouncer
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ע�� debouncer.
*CallBy     :  �κεط����ж������ĳ���
*Input      :  debouncer_function ָ�룬ָ���Ӧ�� debouncer
*Output     :  ��
*Return     :  TRUE���ɹ�.  FLASE��ʧ��.
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT BOOLEAN unregister_debouncer(Debouncer_Data *debouncer_funtion);

/****************************************************************
*Function   :  clear_debouncer
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��� debouncer ��ʷ����.
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT void clear_debouncer(void);

/****************************************************************
*Function   :  get_random
*Author     :  lyy    
*Date       :  2017.4.20
*Description:  ��ȡ����Ƕ�.
*CallBy     :  �κεط����ж������ĳ���
*Input      :  ��
*Output     :  ��
*Return     :  ����Ƕȣ���λ��
*Others     :  
*History    : //�޸���ʷ
    <author>       <time>      <version>           <desc>
    lyy            17.4.28       v1.0         build this function
******************************************************************/
EXPORT U16 get_random(void);

/**************************************************
*���ܣ� ������ת����.
*������ first���Ƿ�������ʼ�Ƕȣ���㣩�����������ʼ�㣬���ѵ�ǰ�Ƕ���Ϊ0�ȣ�
*       rotation_angle����Ҫ��ת�Ķ�������λ��
*       l_speed�������ٶ�
*       r_speed�������ٶ�
*       care_bump���Ƿ��� bump �� cliff
*       result���Ƿ�ת��Ŀ��Ƕ�
*               0���ɹ�
*               1�� ���� cliff
*               2�� ���� bump
*����ֵ����ת�Ƕ�
*
**************************************************/
EXPORT S16 robot_turn(BOOLEAN first, S16 rotation_angle, S16 l_speed,S16 r_speed, BOOLEAN care_bump, S8 *result);

/**************************************************
*���ܣ� ����ֱ�к���.
*������ first���Ƿ��������
*       distance����Ҫֱ�еľ��룬 ��λ����
*       speed���ٶ�
*       care_bump���Ƿ��� bump �� cliff
*       result���Ƿ�ת����Ŀ�ĵ�
*               0���ɹ�
*               1�� ���� cliff
*               2�� ���� bump
*����ֵ��ֱ���˶��پ���
*
**************************************************/
EXPORT S16 robot_drive_go(BOOLEAN first, S16 distance, S16 speed, BOOLEAN care_bump, S8 *result);

/**************************************************
*���ܣ� ���õ�������ʱ�����뵽���λ�õ�ĸ���
*������ num:���뵽���λ�õ�ĸ�������������ã�Ĭ��Ϊ1��
*����ֵ����
**************************************************/
EXPORT void set_navi_no_docksignal(int num);
#endif
