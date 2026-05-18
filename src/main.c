#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "global_def.h"
#include "axis_ctrl.h"
#include "smc_api.h" 

// 辅助函数：阻塞等待运动完成，并实时刷新坐标大屏
void wait_and_print_status() {
    usleep(100000); // 稍微延时，等待指令进入队列
    
    while(SMC_IsParserRunning() || !SMC_IsMotionDone()) {
        double px = SMC_GetLogicalPos(SMC_AXIS_X);
        double py = SMC_GetLogicalPos(SMC_AXIS_Y);
        double pz = SMC_GetLogicalPos(SMC_AXIS_Z);
        double pa = SMC_GetLogicalPos(SMC_AXIS_A);
        int q_cnt = SMC_GetQueueCount();
        
        printf("\r[监控大屏] 队列:%3d | X:%8.3f  Y:%8.3f  Z:%8.3f  A:%8.3f   ", 
               q_cnt, px, py, pz, pa);
        fflush(stdout);
        usleep(50000); // 50ms 刷新率 (20FPS)
    }
    
    // 运动彻底结束，打印最终坐标
    printf("\r[监控大屏] 队列:%3d | X:%8.3f  Y:%8.3f  Z:%8.3f  A:%8.3f   ", 
           0, SMC_GetLogicalPos(SMC_AXIS_X), SMC_GetLogicalPos(SMC_AXIS_Y), 
           SMC_GetLogicalPos(SMC_AXIS_Z), SMC_GetLogicalPos(SMC_AXIS_A));
    printf("\n[提示] 运动已安全完成！\n");
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("用法: sudo %s <EtherCAT网卡名> (例如: sudo ./cnc_core eth0)\n", argv[0]); 
        return 0;
    }

    // ==========================================
    // 1. 系统配置与初始化
    // ==========================================
    axis_sys_init();
    
    SMC_ConfigAxisTopology(SMC_AXIS_X, "X轴", 0, 1, 0);
    SMC_ConfigAxisTopology(SMC_AXIS_Y, "Y轴", 1, 3, 4); // Y轴双驱
    SMC_ConfigGantrySyncAlarm(SMC_AXIS_Y, 1, 1000, 8000, 100); // 放宽的报警阈值
    SMC_ConfigAxisTopology(SMC_AXIS_A, "A轴", 0, 2, 0);
    SMC_ConfigAxisTopology(SMC_AXIS_Z, "Z轴", 0, 5, 0);
    SMC_ConfigSoftLimit(SMC_AXIS_Z, 1, 0.0, 500.0);

    // 启动内核
    if (SMC_InitAndStart(argv[1]) != 0) {
        return -1;
    }

    // ==========================================
    // 2. 开机默认设置全轴原点 (建立 G54 坐标系)
    // ==========================================
    printf("\n[系统] 正在建立初始工件坐标系 (G54)...\n");
    SMC_SetZero(SMC_AXIS_ALL);
    sleep(1);
    printf("[系统] 初始化完毕！准备接受操作员指令。\n");

    // ==========================================
    // 3. 交互式控制台主循环
    // ==========================================
    int choice = -1;
    while (1) 
    {
        printf("\n==============================================\n");
        printf("           SMC 工业数控测试终端           \n");
        printf("==============================================\n");
        printf("  1. 回归原点 (G00 X0 Y0 Z0 A0)\n");
        printf("  2. 加工图纸 (运行 test.txt)\n");
        printf("  3. 相对点动 (手动 JOG 输入坐标)\n");
        printf("  4. 刹车测试 (加工中途触发 Feedhold 平滑暂停)\n");
        printf("  0. 关机退出系统\n");
        printf("----------------------------------------------\n");
        printf("请输入操作编号 (0-4): ");
        
        if (scanf("%d", &choice) != 1) {
            while(getchar() != '\n'); // 清除错误输入缓冲
            continue;
        }

        switch (choice) 
        {
            case 1:
                printf("\n[执行] 正在全轴联动回归原点...\n");
                SMC_GoZero(SMC_AXIS_ALL, 1000.0); // 速度 1000 mm/min
                wait_and_print_status();
                break;

            case 2:
                printf("\n[执行] 开始后台加工图纸 test.txt...\n");
                if (SMC_RunGCodeFile("test.txt") == 0) {
                    wait_and_print_status();
                    printf("\n[成功] ★ 加工完美结束！零件已产出！★\n");
                }
                break;

            case 3:
            {
                int ax = 0;
                double dist = 0.0, spd = 1000.0;
                printf("\n--- 单轴点动面板 ---\n");
                printf("请选择轴号 (0:X, 1:Y, 2:Z, 3:A, -1:全轴): ");
                scanf("%d", &ax);
                printf("请输入移动距离 (mm或度，正负代表方向): ");
                scanf("%lf", &dist);
                printf("请输入移动速度 (mm/min): ");
                scanf("%lf", &spd);
                
                printf("\n[执行] 下发 JOG 指令: 轴[%d] 移动 %.3f, 速度 %.1f...\n", ax, dist, spd);
                SMC_MoveRelative(ax, dist, spd);
                wait_and_print_status();
                break;
            }

            case 4:
                printf("\n[执行] 启动加工，并在 3 秒后触发平滑刹车暂停测试...\n");
                if (SMC_RunGCodeFile("test.txt") == 0) {
                    
                    // 正常跑 3 秒
                    for(int i=0; i<30; i++) {
                        printf("\r[正常加工] X:%8.3f Y:%8.3f  ", SMC_GetLogicalPos(0), SMC_GetLogicalPos(1));
                        fflush(stdout);
                        usleep(100000);
                    }
                    
                    // 触发暂停！体验基于虚拟时间轴的平滑减速
                    printf("\n\n⚠️ [干预] 收到进给保持(Feedhold)指令！系统正在平滑刹车...\n");
                    SMC_PauseProcessing(); 
                    
                    // 停滞观察 3 秒
                    for(int i=0; i<30; i++) {
                        printf("\r[暂停停稳] X:%8.3f Y:%8.3f (机床锁定中) ", SMC_GetLogicalPos(0), SMC_GetLogicalPos(1));
                        fflush(stdout);
                        usleep(100000);
                    }
                    
                    // 恢复运行
                    printf("\n\n✅ [干预] 收到恢复(Resume)指令！系统正在平滑加速恢复轨迹...\n");
                    SMC_ResumeProcessing();
                    
                    // 把剩下的跑完
                    wait_and_print_status();
                }
                break;

            case 0:
                printf("\n[退出] 收到关机指令，准备断开伺服动力...\n");
                SMC_Close();
                return 0;

            default:
                printf("\n[错误] 无效的指令编号！\n");
                break;
        }
    }

    return 0;
}