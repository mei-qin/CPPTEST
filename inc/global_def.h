#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H

#include "soem/soem.h"
#include "axis_cfg.h"

/************************ SOEM全局核心变量（b3_drive原有） ************************/
extern ecx_contextt ctx;                // SOEM主站上下文
extern uint8 IOmap[4096];               // PDO映射缓冲区（所有轴共用）
extern OSAL_THREAD_HANDLE thread_rt;    // 实时线程句柄
extern OSAL_THREAD_HANDLE thread_chk;   // 故障检查线程句柄

extern int expectedWKC;                 // SOEM期望WKC值
extern int wkc;                         // 实际WKC值
extern int mappingdone, dorun, inOP;    // SOEM状态标志
extern int dowkccheck, currentgroup;    // SOEM故障检查标志

/************************ 实时控制全局变量 ************************/
extern int cycle;                       // 实时周期计数
extern int64 cycletime;                 // 实时周期（1ms）
extern int64 syncoffset, timeerror;     // DC同步参数
extern float pgain, igain;              // DC同步PI参数

/************************ 五轴全局核心变量 ************************/
extern AxisCtrl_t g_axis[AXIS_NUM];     // 五轴控制数组（核心！）
extern int g_all_axis_op_ready;         // 全局标志：五轴均使能就绪
extern int g_all_axis_reach;            // 全局标志：五轴均目标到达

#endif // GLOBAL_DEF_H
