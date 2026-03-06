#include "axis_ctrl.h"
#include "global_def.h"
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>

/************************ 全局变量定义（仅轴相关，其余在ecat_core.c） ************************/
AxisCtrl_t g_axis[AXIS_NUM];            // 五轴核心数组，全局唯一定义
int g_all_axis_op_ready = 0;            // 五轴均使能就绪标志
int g_all_axis_reach = 0;               // 五轴均目标到达标志

/************************ 五轴系统初始化（核心配置，修改此函数即可调整轴参数） ************************/
void axis_sys_init(void)
{
    // 轴0：X轴，从站ID=1，PP目标位置=20000
    g_axis[0].slave_id = 1;
    strcpy(g_axis[0].axis_name, "X轴");
    g_axis[0].pp_target_pos = 30000;

    g_axis[1].slave_id = 2;
    strcpy(g_axis[1].axis_name, "Y轴");
    g_axis[1].pp_target_pos = 20000;

    g_axis[2].slave_id = 3;
    strcpy(g_axis[2].axis_name, "Z轴");
    g_axis[2].pp_target_pos = 30000;

    // 初始化所有轴的动态状态为默认值
    for (int i = 0; i < AXIS_NUM; i++)
    {
        g_axis[i].cia_step = 0;
        g_axis[i].pp_trigger_sent = 0;
        g_axis[i].is_op_ready = 0;
        g_axis[i].is_target_reach = 0;
        g_axis[i].is_error = 0;
    }
    g_all_axis_op_ready = 0;
    g_all_axis_reach = 0;
    printf("[五轴初始化] 5轴参数配置完成，从站ID：1(X)、2(Y)、3(Z)、4(A)、5(B)\n");
}

/************************ 单轴SDO配置PP模式 ************************/
void axis_sdo_config_pp(int axis_idx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM)
    {
        printf("[SDO错误] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return;
    }
    if (g_axis[axis_idx].is_error) return; // 故障轴跳过

    uint8 pp_mode = 0x01; // CiA402 PP模式=1
    int sz = sizeof(pp_mode);
    ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, sz, &pp_mode, EC_TIMEOUTRXM);
    osal_usleep(200000);
    /*if (ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, sz, &pp_mode, EC_TIMEOUTRXM) == 0)
    {
        printf("[SDO配置] %s PP模式配置成功(0x6060=1)\n", g_axis[axis_idx].axis_name);
    }
    else
    {
        printf("[SDO错误] %s PP模式配置失败！请检查从站ID和SAFE_OP状态\n", g_axis[axis_idx].axis_name);
        g_axis[axis_idx].is_error = 1;
        exit(-1); // 配置失败直接退出，
    }*/
}

/************************ 单轴PDO写（控制字+目标位置） ************************/
void axis_pdo_write(int axis_idx, uint16 cw, int32 pos)
{
    // 越界检查
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    if (g_axis[axis_idx].is_error) return;

    // 空指针检查+故障置位
    uint8 *out = ctx.slavelist[g_axis[axis_idx].slave_id].outputs;
    if (!out)
    {
        g_axis[axis_idx].is_error = 1;
        printf("[PDO错误] %s PDO输出缓冲区为空！\n", g_axis[axis_idx].axis_name);
        return;
    }

    // 写入控制字（2字节，小端）
    out[PDO_CW_BYTE0] = cw & 0xFF;
    out[PDO_CW_BYTE1] = (cw >> 8) & 0xFF;

    // 写入目标位置（4字节，小端，台达B3-E标准）
    out[PDO_POS_BYTE0] = pos & 0xFF;
    out[PDO_POS_BYTE1] = (pos >> 8) & 0xFF;
    out[PDO_POS_BYTE2] = (pos >> 16) & 0xFF;
    out[PDO_POS_BYTE3] = (pos >> 24) & 0xFF;
}

/************************ 单轴PDO读（状态字） ************************/
uint16 axis_pdo_read_sw(int axis_idx)
{
    // 越界检查
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return 0;
    if (g_axis[axis_idx].is_error) return 0;

    // 空指针检查+故障置位
    uint8 *in = ctx.slavelist[g_axis[axis_idx].slave_id].inputs;
    if (!in)
    {
        g_axis[axis_idx].is_error = 1;
        printf("[PDO错误] %s PDO输入缓冲区为空！\n", g_axis[axis_idx].axis_name);
        return 0;
    }

    // 读取状态字（2字节，高位在后）
    return (uint16)(in[PDO_SW_BYTE1] << 8 | in[PDO_SW_BYTE0]);
}
