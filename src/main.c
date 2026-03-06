#include <stdio.h>
#include <stdlib.h>
#include "axis_ctrl.h"
#include "ecat_core.h"
#include "global_def.h"

/************************ 程序入口（极简，仅做初始化和启动） ************************/
int main(int argc, char *argv[])
{
    // 1. 检查命令行参数（指定网卡，如eth0）
    if (argc < 2)
    {
        printf("Usage: %s <EtherCAT网卡名>\n", argv[0]);
        printf("Example: %s enp7s0\n", argv[0]);
        return 0;
    }

    // 2. 五轴系统参数初始化（配置每个轴的核心参数）
    axis_sys_init();

    // 3. 创建实时线程+故障检查线程（适配SOEM的OSAL线程库）
    if (!osal_thread_create_rt(&thread_rt, 128000, &ecat_thread_rt, NULL))
    {
        printf("[线程错误] 实时控制线程创建失败！\n");
        return -1;
    }
    if (!osal_thread_create(&thread_chk, 128000, &ecat_thread_chk, NULL))
    {
        printf("[线程错误] 故障检查线程创建失败！\n");
        return -1;
    }
    printf("[线程] 实时线程+故障检查线程创建成功\n");

    // 4. ECAT主站一键启动（包含所有初始化步骤）
    ecat_bringup(argv[1]);

    // 5. 主循环：等待五轴均目标到达，然后正常终止
    while (!g_all_axis_reach)
    {
        osal_usleep(100000); // 100ms轮询，不占用CPU
    }

    // 6. 程序正常终止，释放资源（工业级规范）
    printf("\n[五轴系统] 所有轴目标到达，程序开始正常终止！\n");
    dorun = 0; // 通知实时线程，所有轴失能
    osal_usleep(2000000); // 延时2ms，让实时线程执行失能逻辑

    for (int i = 0; i < AXIS_NUM; i++) {
       if (g_axis[i].slave_id > 0) {
         axis_pdo_write(i, CW_ENABLE_OP ,0);
       }
    }
    ecx_send_processdata(&ctx);
    osal_usleep(50000); // 50ms延时，确保状态切换

    for (int i = 0; i < AXIS_NUM; i++) {
       if (g_axis[i].slave_id > 0) {
         axis_pdo_write(i, CW_SHUTDOWN, 0);
       }
    }
    ecx_send_processdata(&ctx);
    osal_usleep(100000); // 100ms延时，确认伺服失能完成

    int shutdown_ok = 1;
    for (int i = 0; i < AXIS_NUM; i++) {
       if (g_axis[i].slave_id > 0) {
         uint16 sw = axis_pdo_read_sw(i);
         if ((sw & SW_MASK) != SW_SHUTDOWN_RDY) {
           printf("[退出警告] %s 未进入SHUTDOWN_RDY，当前状态字：0x%04X\n", g_axis[i].axis_name, sw);
           shutdown_ok = 0;
           }
         }
    }
    if (shutdown_ok) {
      printf("[退出流程] 所有轴已进入SHUTDOWN_RDY\n");
    }

    // ========== 步骤2：EtherCAT状态降级（OP→SAFE_OP→INIT） ==========
    // 第一步：全局切换到SAFE_OP并验证
    printf("[ECAT] 切换所有从站到SAFE_OP状态...\n");
    ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ctx, 0); // 全局设置SAFE_OP
    // 验证是否切换成功（和b3_drive一致，带超时检查）
    if (ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
      printf("[ECAT警告] 切换SAFE_OP失败，但继续执行降级流程\n");
    }

    // 第二步：全局切换到INIT并验证
    printf("[ECAT] 切换所有从站到INIT状态...\n");
    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0); // 全局设置INIT
    if (ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE) != EC_STATE_INIT) {
        printf("[ECAT警告] 切换INIT失败，但继续关闭主站\n");
    }
    // ========== 步骤3：最后关闭主站 ==========
    ecx_close(&ctx);      // 关闭SOEM主站，释放网卡资源

    printf("[五轴系统] 程序正常退出！\n");
    return 0;
}
