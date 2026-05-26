#ifndef SMC_API_H
#define SMC_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 轴号定义
#define SMC_AXIS_X    0
#define SMC_AXIS_Y    1
#define SMC_AXIS_Z    2
#define SMC_AXIS_A    3
#define SMC_AXIS_B    4
#define SMC_AXIS_ALL -1

// =======================================================
// 系统生命周期管理 API
// =======================================================
// 初始化并启动系统 (包含 EtherCAT 组网、线程启动、伺服上电握手)
// 阻塞函数，直到全轴就绪返回 0，失败返回 -1
int SMC_InitAndStart(const char *netif_name);

// 安全关闭系统 (伺服优雅下电，释放网卡)
void SMC_Close(void);

// =======================================================
// 轴配置 API
// =======================================================
// 1.配置轴的从站拓扑结构（单驱/双驱）和名称
// is_dual_drive: 0=单驱，1=双驱
// master_id/slave_id: 双驱时分别指定主从ID
int SMC_ConfigAxisTopology(int axis_idx, const char* axis_name,int is_dual_drive,int master_id,int slave_id);

// 2.配置轴的软件限位参数
int SMC_ConfigSoftLimit(int axis_idx,int enable,double neg_limit_mm,double pos_limit_mm);

// 3.配置轴的龙门同步报警参数
int SMC_ConfigGantrySyncAlarm(int axis_idx,int enable,int32_t tolerance_pulse,int32_t max_error_pulse,int time_ms);


// =======================================================
// 坐标与状态获取 API
// =======================================================
// 获取指定轴的当前逻辑坐标 (单位: 脉冲)
double SMC_GetLogicalPos(int axis_idx);

// 获取 G 代码解析器状态 (1:正在解析/加工中, 0:空闲)
int SMC_IsParserRunning(void);

// 判断底层运动是否彻底结束 (队列空 且 插补器停止)
int SMC_IsMotionDone(void);

// 获取底层 FIFO 队列堆积的指令数量 (用于UI进度条)
int SMC_GetQueueCount(void);


// =======================================================
// 运动控制 API
// =======================================================
// 设定原点 (工件坐标系 G54)
void SMC_SetZero(int axis_idx);

// 单轴相对点动 (JOG)
void SMC_MoveRelative(int axis_idx, double distance, double speed);

// 多轴联动回归原点
void SMC_GoZero(int axis_idx, double speed);

// =======================================================
// G 代码加工 API
// =======================================================
// 启动后台加工 (非阻塞，下发后立刻返回)
int SMC_RunGCodeFile(const char *filepath);

// 暂停加工 (Hold)
void SMC_PauseProcessing(void);

// 恢复加工 (Resume)
void SMC_ResumeProcessing(void);

// 紧急中止 (Abort)
void SMC_AbortProcessing(void);

// 配置轴的脉冲/单位 (例如 脉冲/mm 或 脉冲/度)，用于位置/速度换算
void SMC_ConfigPulsePerUnit(int axis_idx,double pulse_per_unit);


#ifdef __cplusplus
}
#endif

#endif // SMC_API_H