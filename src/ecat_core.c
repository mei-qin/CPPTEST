#include "ecat_core.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include <stdio.h>
#include <stdlib.h>

/************************ 全局变量定义（SOEM+实时控制相关） ************************/
// SOEM核心变量
ecx_contextt ctx;
uint8 IOmap[4096];
OSAL_THREAD_HANDLE thread_rt;
OSAL_THREAD_HANDLE thread_chk;

int expectedWKC;
int wkc;
int mappingdone = 0, dorun = 0, inOP = 0;
int dowkccheck = 0, currentgroup = 0;

// 实时控制变量
int cycle = 0;
int64 cycletime = CYCLE_TIME_NS;
int64 syncoffset = 500000;
int64 timeerror;
float pgain = 0.01f;
float igain = 0.00002f;

/************************ 时间工具：纳秒级时间累加（b3_drive原有） ************************/
void add_time_ns(ec_timet *ts, int64 addtime)
{
    ec_timet addts;
    addts.tv_nsec = addtime % NSEC_PER_SEC;
    addts.tv_sec  = (addtime - addts.tv_nsec) / NSEC_PER_SEC;
    osal_timespecadd(ts, &addts, ts);
}

/************************ DC同步PI函数（适配SOEM旧版本-DCcycle） ************************/
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;

    delta = (reftime - syncoffset) % cycletime;
    if (delta > (cycletime / 2))
        delta -= cycletime;

    timeerror = -delta;
    integral += timeerror;
    *offsettime = (int64)((timeerror * pgain) + (integral * igain));
}

/************************ 实时控制线程（核心！1ms周期，五轴同周期控制） ************************/
OSAL_THREAD_FUNC_RT ecat_thread_rt(void *arg)
{
    ec_timet ts;
    int ht;
    static int64_t toff = 0;
    uint16 sw = 0; // 临时缓存状态字，减少重复读取

    // b3_drive原有初始化逻辑，无需修改
    dorun = 0;
    while (!mappingdone) osal_usleep(100);
    osal_get_monotonic_time(&ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000; // 校准到毫秒整，消除启动延时
    ecx_send_processdata(&ctx);
    printf("[实时线程] 初始化完成，等待dorun启动\n");

    while (1)
    {
        // 1ms实时定时（五轴共用，b3_drive原有）
        add_time_ns(&ts, cycletime + toff);
        osal_monotonic_sleep(&ts);

        if (dorun > 0)
        {
            cycle++;
            // 读取所有轴PDO数据（SOEM底层批量读取）
            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

            // DC时钟同步（适配SOEM旧版本，五轴共用同步时钟）
            if (ctx.slavelist[0].hasdc && (wkc > 0))
                ec_sync(ctx.slavelist[0].DCcycle, cycletime, &toff);

            for (int i = 0; i < AXIS_NUM; i++)
         {
                if (g_axis[i].is_error) continue;
                sw = axis_pdo_read_sw(i); // 读当前轴状态字
                g_axis[i].cia_step_delay++; // 每个周期累加延时计数器

                switch (g_axis[i].cia_step)
                {
                    case 0: // 步骤0：故障复位 → 持续发送+50ms延时
                        axis_pdo_write(i, CW_FAULT_RESET, 0); // 持续发送复位指令
                        // 延时50个周期（50ms）+ 故障清除校验
                        if (g_axis[i].cia_step_delay > 50 && ((sw & 0x0008) == SW_FAULT_CLEAR))
                        {
                            printf("[CiA402] %s 故障复位完成\n", g_axis[i].axis_name);
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay = 0; // 重置延时
                        }
                        break;

                    case 1: // 步骤1：关机 → 持续发送+50ms延时
                        axis_pdo_write(i, CW_SHUTDOWN, 0); // 持续发送关机指令
                        if (g_axis[i].cia_step_delay > 50 && ((sw & SW_MASK) == SW_SHUTDOWN_RDY))
                        {
                            printf("[CiA402] %s 关机就绪\n", g_axis[i].axis_name);
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay = 0;
                        }
                        break;

                    case 2: // 步骤2：上电 → 持续发送+50ms延时
                        axis_pdo_write(i, CW_SWITCH_ON, 0); // 持续发送上电指令
                        if (g_axis[i].cia_step_delay > 50 && ((sw & SW_MASK) == SW_SWITCHED_ON))
                        {
                            printf("[CiA402] %s 上电完成\n", g_axis[i].axis_name);
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay = 0;
                        }
                        break;

                    case 3: // 步骤3：操作使能 → 持续发送+50ms延时
                        axis_pdo_write(i, CW_ENABLE_OP, 0); // 持续发送使能指令
                        if (g_axis[i].cia_step_delay > 50 && ((sw & SW_MASK) == SW_OP_ENABLED))
                        {
                            printf("[CiA402] %s 操作使能就绪\n", g_axis[i].axis_name);
                            g_axis[i].is_op_ready = 1;
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay = 0;
                        }
                        break;

                    case 4: // 步骤4：等待全局同步 → 持续发使能+等待所有轴就绪
                        axis_pdo_write(i, CW_ENABLE_OP, 0); // 持续发送使能指令
                        if (g_all_axis_op_ready)
                        {
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay = 0;
                        }
                        break;

                    case 5: // 步骤5：PP运动触发 → 防止重复触发+持续发指令
                        if (!g_axis[i].pp_trigger_sent)
                        {
                            axis_pdo_write(i, CW_PP_TRIGGER, g_axis[i].pp_target_pos);
                            g_axis[i].pp_trigger_sent = 1;
                            printf("[PP运动] %s 触发运动，目标位置：%d\n", g_axis[i].axis_name, g_axis[i].pp_target_pos);
                        }
                        else
                        {
                            axis_pdo_write(i, CW_ENABLE_OP, g_axis[i].pp_target_pos); // 持续发送保持
                        }
                        // 检查目标到达
                        if (sw & SW_TARGET_REACH)
                        {
                            g_axis[i].is_target_reach = 1;
                            printf("[PP到位] %s 目标到达，当前周期：%d\n", g_axis[i].axis_name, cycle);
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay = 0;
                        }
                        break;

                    default: // 步骤6：运动完成，保持使能
                        axis_pdo_write(i, CW_ENABLE_OP, g_axis[i].pp_target_pos);
                        if (cycle % 1000 == 0) 
                        {
                           printf("[CiA402] %s 运动完成，持续发送使能指令\n", g_axis[i].axis_name);
                        }                   

                        break;
                }
            }

            // 2. 更新全局使能就绪标志：所有轴均使能才触发运动
            g_all_axis_op_ready = 1;
            for (int i = 0; i < AXIS_NUM; i++)
            {
                if (!g_axis[i].is_op_ready || g_axis[i].is_error)
                {
                    g_all_axis_op_ready = 0;
                    break;
                }
            }

            // 3. 更新全局目标到达标志：所有轴均到位才终止程序
            g_all_axis_reach = 1;
            for (int i = 0; i < AXIS_NUM; i++)
            {
                if (g_axis[i].slave_id > 0 && (!g_axis[i].is_target_reach || g_axis[i].is_error))
                {
                    g_all_axis_reach = 0;
                    break;
                }
            }

            // SOEM底层：处理邮箱+批量发送所有轴PDO数据
            ecx_mbxhandler(&ctx, 0, 4);
            ecx_send_processdata(&ctx);
        }
        else
        {
            // dorun=0时，所有轴失能（工业级规范，防止伺服励磁）
            for (int i = 0; i < AXIS_NUM; i++)
            {
                axis_pdo_write(i, CW_SHUTDOWN, 0);
            }
            ecx_send_processdata(&ctx);
        }
    }
    return ;
}

/************************ 故障检查线程（非实时，10ms周期，不影响实时性） ************************/
OSAL_THREAD_FUNC ecat_thread_chk(void *arg)
{
    int slaveix;
    while (1)
    {
        if (inOP && ((dowkccheck > 2) || ctx.grouplist[currentgroup].docheckstate))
        {
            ctx.grouplist[currentgroup].docheckstate = FALSE;
            ecx_readstate(&ctx); // 读取所有从站状态

            // 遍历五轴，检查是否在OP状态，非OP则置故障标志
            for (int i = 0; i < AXIS_NUM; i++)
            {
                slaveix = g_axis[i].slave_id;
                ec_slavet *slave = &ctx.slavelist[slaveix];
                if (slave->state != EC_STATE_OPERATIONAL)
                {
                    ctx.grouplist[currentgroup].docheckstate = TRUE;
                    g_axis[i].is_error = 1;
                    printf("[故障检测] %s 非OP状态！当前状态码：%d\n", g_axis[i].axis_name, slave->state);
                }
            }
            dowkccheck = 0;
        }
        osal_usleep(10000); // 10ms检测一次，非实时，降低CPU占用
    }
    return ;
}

/************************ ECAT主站启动封装（核心！一键完成所有初始化） ************************/
void ecat_bringup(char *ifname)
{
    // 1. SOEM主站初始化+容错
    if (!ecx_init(&ctx, ifname))
    {
        printf("[ECAT错误] 主站初始化失败！网卡：%s\n", ifname);
        exit(-1);
    }
    printf("[ECAT] 主站初始化成功，网卡：%s\n", ifname);

    // 2. 扫描EtherCAT从站+容错
    if (ecx_config_init(&ctx) <= 0)
    {
        printf("[ECAT错误] 未扫描到任何EtherCAT从站！\n");
        ecx_close(&ctx);
        exit(-1);
    }
    printf("[ECAT] 扫描到%d个EtherCAT从站\n", ctx.slavecount);

    // 3. PDO映射+DC时钟配置（SOEM底层批量处理）
    ecx_config_map_group(&ctx, IOmap, 0);
    ecx_configdc(&ctx);
    expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;
    printf("[ECAT] PDO映射+DC配置完成，期望WKC值：%d\n", expectedWKC);

    // 4. 所有轴切换到SAFE_OP状态（SDO配置必备，EtherCAT标准）
    for (int i = 0; i < AXIS_NUM; i++)
    {
        ctx.slavelist[g_axis[i].slave_id].state = EC_STATE_SAFE_OP;
        ecx_writestate(&ctx, g_axis[i].slave_id);
        if (ecx_statecheck(&ctx, g_axis[i].slave_id, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP)
        {
            printf("[ECAT错误] %s 无法进入SAFE_OP状态！\n", g_axis[i].axis_name);
            ecx_close(&ctx);
            exit(-1);
        }
    }
    printf("[ECAT] 所有轴已进入SAFE_OP状态，开始SDO配置\n");

    // 5. 所有轴SDO配置PP模式
    for (int i = 0; i < AXIS_NUM; i++)
    {
        axis_sdo_config_pp(i);
    }

    // 6. 置映射完成标志，启动实时线程
    mappingdone = 1;
    dorun = 1;
    osal_usleep(1000000); // 延时1ms，让实时线程完成初始化

    // 7. 全局切换到OP状态（所有从站进入运行状态）
    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    if (ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
    {
        printf("[ECAT错误] 无法进入全局OP状态！\n");
        ecx_close(&ctx);
        exit(-1);
    }
    inOP = 1;
    printf("[ECAT] 全局OP状态进入成功，五轴伺服系统启动完成！\n");
}
