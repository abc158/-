/*
*********************************************************************************************************
* Filename      : app_cfg.h
* Version       : V1.00
*********************************************************************************************************
*/
#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__

//process prio
#define  APP_TASK_MAIN_PRIO                  5/*app-main*/
#define  APP_TASK_RVE1                       6/*保留*/
#define  APP_TASK_DOCK_PRIO                  7/*dock,这个不能修改*/

//stack space
#define  APP_TASK_STK_SIZE             128

#endif
