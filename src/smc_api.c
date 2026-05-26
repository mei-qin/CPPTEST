#include "smc_api.h"
#include "global_def.h"
#include "ecat_core.h"
#include "axis_ctrl.h"
#include "gcode_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// ================== 系统管理 ==================
int SMC_InitAndStart(const char *netif_name)
{
    printf("[SMC_API] 正在初始化运动控制内核...\n");
    
    if (!osal_thread_create_rt(&thread_rt, 128000, &ecat_thread_rt, NULL)) {
        printf("[SMC_API] 实时控制线程创建失败！\n"); return -1;
    }
    if (!osal_thread_create(&thread_chk, 128000, &ecat_thread_chk, NULL)) {
        printf("[SMC_API] 故障检查线程创建失败！\n"); return -1;
    }
    if(!osal_thread_create(&thread_parser, 128000, &parser_thread_func, NULL)){
        printf("[SMC_API] G-code解析线程创建失败！\n"); return -1;
    }

    ecat_bringup((char*)netif_name);

    printf("[SMC_API] 等待伺服全轴使能并进入 CSP 同步...\n");
    int timeout = 50; // 最多等5秒
    while (!g_all_axis_op_ready && timeout > 0) {
        osal_usleep(100000);
        timeout--;
    }
    if (timeout == 0) {
        printf("[SMC_API] 伺服使能超时！请检查硬件。\n"); return -1;
    }
    
    // 给系统时钟收敛留时间
    osal_usleep(1000000); 
    printf("[SMC_API] ★ 内核启动完毕，伺服就绪！★\n");
    return 0;
}

void SMC_Close(void)
{
    printf("\n[SMC_API] 收到关闭请求，执行安全下电流程...\n");
    dorun = 0; // 通知实时线程切断动力
    osal_usleep(2000000); 

    // 关机流程 
    for (int i = 0; i < AXIS_NUM; i++) {
      for(int s=0;s<g_axis[i].slave_count;s++){
         axis_pdo_write(g_axis[i].slave_ids[s], CW_SWITCH_ON ,0); 
       }
    }
    ecx_send_processdata(&ctx);
    osal_usleep(50000); 

    for (int i = 0; i < AXIS_NUM; i++) {
      for(int s=0;s<g_axis[i].slave_count;s++){
         axis_pdo_write(g_axis[i].slave_ids[s], CW_SHUTDOWN ,0); 
       }
    }
    ecx_send_processdata(&ctx);
    osal_usleep(100000); 

    // 降级与关闭
    ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ctx, 0);
    if (ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
        printf("[ECAT警告] 切换SAFE_OP失败，但继续执行降级流程\n");
    }
    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0);     
    if (ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE) != EC_STATE_INIT) {
        printf("[ECAT警告] 切换INIT失败，但继续关闭主站\n");
    }
    ecx_close(&ctx); 
    printf("[SMC_API] 系统已安全关闭！\n");
}

// ================== 坐标与状态 ==================
double SMC_GetLogicalPos(int axis_idx) {
    if(axis_idx < 0 || axis_idx >= AXIS_NUM) return 0;
    return g_coord_mgr.current_logical_pos[axis_idx]; 
}

int SMC_IsParserRunning() { return g_parser_ctrl.is_running; }
int SMC_IsMotionDone() { return is_trajectory_finished(); }
int SMC_GetQueueCount() {
    return (g_cmd_queue.head - g_cmd_queue.tail + QUEUE_SIZE) % QUEUE_SIZE;
}

// ================== 运动控制 ==================
void SMC_SetZero(int axis_idx) { api_set_zero(axis_idx); }
void SMC_MoveRelative(int axis_idx, double distance, double speed) { api_move_relative(axis_idx, distance, speed); }
void SMC_GoZero(int axis_idx, double speed) { api_go_zero(axis_idx, speed); }

// ================== G代码加工 ==================
int SMC_RunGCodeFile(const char *filepath) {
    if (g_parser_ctrl.is_running) return -1; // 正在加工，拒绝
    strncpy(g_parser_ctrl.filepath, filepath, sizeof(g_parser_ctrl.filepath)-1);
    g_parser_ctrl.abort_request = 0;
    g_parser_ctrl.is_paused = 0;
    g_parser_ctrl.is_running = 1; // 触发 Parser 线程
    return 0;
}

void SMC_AbortProcessing() { g_parser_ctrl.abort_request = 1; }



int SMC_ConfigAxisTopology(int axis_idx, const char* name,int is_dual_drive,int master_id,int slave_id){
    if(axis_idx < 0 || axis_idx >= AXIS_NUM) {
        printf("[SMC_API] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return -1;
    }
    strncpy(g_axis[axis_idx].axis_name,name, 15);
    if(is_dual_drive){
        g_axis[axis_idx].slave_ids[0]=master_id;
        g_axis[axis_idx].slave_ids[1]=slave_id;
        g_axis[axis_idx].slave_count=2;
        printf("[SMC_API] 配置 %s 为双驱，主ID: %d, 从ID: %d\n", name, master_id, slave_id);
    }else{
        g_axis[axis_idx].slave_ids[0]=master_id;
        g_axis[axis_idx].slave_count=1;
        printf("[SMC_API] 配置 %s 为单驱，主ID: %d\n", name, master_id);
    }
    return 0;
}

int SMC_ConfigSoftLimit(int axis_idx,int enable,double neg_limit_mm,double pos_limit_mm){
    if(axis_idx < 0 || axis_idx >= AXIS_NUM) {
        printf("[SMC_API] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return -1;
    }
    g_axis[axis_idx].enable_soft_limit = enable;
    g_axis[axis_idx].soft_limit_neg= neg_limit_mm;
    g_axis[axis_idx].soft_limit_pos = pos_limit_mm;
    return 0;
}

int SMC_ConfigGantrySyncAlarm(int axis_idx,int enable,int32_t tolerance_pulse,int32_t max_error_pulse,int time_ms){
    if(axis_idx < 0 || axis_idx >= AXIS_NUM) {
        printf("[SMC_API] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return -1;
    }
    g_axis[axis_idx].enable_sync_alarm = enable;
    g_axis[axis_idx].sync_tolerance_pulse= tolerance_pulse;
    g_axis[axis_idx].sync_max_err_pulse = max_error_pulse;
    g_axis[axis_idx].sync_err_time_ms = time_ms;
    g_axis[axis_idx]._current_sync_timer=0;
    return 0;
}

void SMC_PauseProcessing(){
    g_parser_ctrl.is_paused=1;
    api_motion_pause();
}

void SMC_ResumeProcessing(){
    g_parser_ctrl.is_paused=0;
    api_motion_resume();
}

void SMC_ConfigPulsePerUnit(int axis_idx,double pulse_per_unit){
    if(axis_idx < 0 || axis_idx >= AXIS_NUM) {
        printf("[SMC_API] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return;
    }
    g_axis[axis_idx].pulse_per_unit=pulse_per_unit;
}