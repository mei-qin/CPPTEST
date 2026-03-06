#ifndef AXIS_CFG_H
#define AXIS_CFG_H

#include <stdint.h>
#include <string.h>
typedef int32_t int32;
typedef uint16_t uint16;
/************************ 核心宏定义 ************************/
#define AXIS_NUM         3       // 五轴核心宏！改此值可灵活增减轴数
#define EC_TIMEOUTMON    500     // SOEM超时时间
#define NSEC_PER_SEC     1000000000
#define CYCLE_TIME_NS    1000000  // 1ms实时周期，五轴共用

/************************ CiA402 控制字（台达B3-E标准） ************************/
#define CW_FAULT_RESET   0x0080
#define CW_SHUTDOWN      0x0006
#define CW_SWITCH_ON     0x0007
#define CW_ENABLE_OP     0x000F
#define CW_PP_TRIGGER    0x001F   // bit4上升沿触发PP运动

/************************ CiA402 状态字（掩码+标准状态） ************************/
#define SW_MASK          0x006F   // CiA402状态字有效位掩码
#define SW_FAULT_CLEAR   0x0000   // 故障位(bit3)清零
#define SW_SHUTDOWN_RDY  0x0021   // 关机就绪
#define SW_SWITCHED_ON   0x0023   // 伺服上电完成
#define SW_OP_ENABLED    0x0027   // 操作使能就绪
#define SW_TARGET_REACH  0x0400   // 目标位置到达

/************************ PDO 字节偏移（台达B3-E标准PP模式，所有轴统一） ************************/
#define PDO_CW_BYTE0     0
#define PDO_CW_BYTE1     1
#define PDO_POS_BYTE0    2
#define PDO_POS_BYTE1    3
#define PDO_POS_BYTE2    4
#define PDO_POS_BYTE3    5
#define PDO_SW_BYTE0     0
#define PDO_SW_BYTE1     1

// 补全所有未定义的对象字典标识符
#define OD6060_MODE_OF_OPERATION 0x6060  // 操作模式
#define OD607A_TARGET_POSITION   0x607A  // 目标位置（32位有符号）
#define OD6081_MAX_SPEED         0x6081  // 最大速度（32位无符号）
#define OD6083_ACCELERATION      0x6083  // 加速时间（32位无符号）
#define OD6084_DECELERATION      0x6084  // 减速时间（32位无符号）
#define OD6040_CONTROL_WORD      0x6040  // 控制字（备用）
#define OD6041_STATUS_WORD       0x6041  // 状态字（备用）
#define OD60FF_TARGET_SPEED      0x60FF  // 目标速度（32位有符号）

/************************ 轴参数结构体（单轴所有参数独立封装） ************************/
// 每个轴的独立配置/状态，五轴通过数组管理，修改单轴不影响其他轴
typedef struct {
    // ① 静态配置（初始化赋值，运行中不变）
    int slave_id;          // 该轴EtherCAT从站ID（1-5）
    char axis_name[16];    // 轴名（X/Y/Z/A/B，调试友好）
    int32 pp_target_pos;   // PP模式目标位置（脉冲/PUU）
    // ② 动态状态（运行时实时更新，每个轴独立）
    int cia_step;          // CiA402状态机步骤（0-6）
    int cia_step_delay;    // 步骤延时计数器
    int pp_trigger_sent;   // PP触发标志（0=未触发，1=已触发）
    int is_op_ready;       // 操作使能就绪（0=否，1=是）
    int is_target_reach;   // 目标到达（0=否，1=是）
    int is_error;          // 故障标志（0=无错，1=故障）
} AxisCtrl_t;

#endif // AXIS_CFG_H
