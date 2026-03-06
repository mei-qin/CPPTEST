#ifndef AXIS_CTRL_H
#define AXIS_CTRL_H

#include "axis_cfg.h"

/************************ 五轴核心控制函数声明 ************************/
// 1. 五轴系统初始化：配置每个轴的从站ID、轴名、目标位置（核心配置）
void axis_sys_init(void);

// 2. 单轴SDO配置：给指定轴配置CiA402 PP模式（0x6060=1）
void axis_sdo_config_pp(int axis_idx);

// 3. 单轴PDO写：写入控制字+目标位置
void axis_pdo_write(int axis_idx, uint16 cw, int32 pos);

// 4. 单轴PDO读：读取状态字
uint16 axis_pdo_read_sw(int axis_idx);

#endif // AXIS_CTRL_H
