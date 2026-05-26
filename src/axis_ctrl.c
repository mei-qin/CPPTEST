#include "axis_ctrl.h"
#include "global_def.h"
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "planner.h"
#include "gcode_parser.h"
/************************ 全局变量定义（仅轴相关，其余在ecat_core.c） ************************/
AxisCtrl_t g_axis[AXIS_NUM];            // 五轴核心数组，全局唯一定义
int g_all_axis_op_ready = 0;            // 五轴均使能就绪标志
int g_all_axis_reach = 0;               // 五轴均目标到达标志
UI_Command_t g_ui_cmd={CMD_NONE,0,0,0,0,0,0};
Interpolator_t g_interpolator={0};
CommandQueue_t g_cmd_queue={0};
static double plan_cursor[AXIS_NUM]={0};
CoordManager_t g_coord_mgr={COORD_G54,{0},{0},{0}};
/************************ 五轴系统初始化（核心配置，修改此函数即可调整轴参数） ************************/
static void wait_cmd_accepted(){
    while (g_ui_cmd.execute==1)
    {
       osal_usleep(1000);
    }  
}
static int check_soft_limits(double target_x,double target_y,double target_z){
    // Implementation for soft limit checking
    if(g_axis[AXIS_X].enable_soft_limit){
        if(target_x>g_axis[AXIS_X].soft_limit_pos||target_x<g_axis[AXIS_X].soft_limit_neg){
            printf("[软限位] X轴目标位置%.2f超出软限位范围！\n", target_x);
            return 0;
        }
    }
    if(g_axis[AXIS_Y].enable_soft_limit){
        if(target_y>g_axis[AXIS_Y].soft_limit_pos||target_y<g_axis[AXIS_Y].soft_limit_neg){
            printf("[软限位] Y轴目标位置%.2f超出软限位范围！\n", target_y);
            return 0;
        }
    }
    if(g_axis[AXIS_Z].enable_soft_limit){
        if(target_z>g_axis[AXIS_Z].soft_limit_pos||target_z<g_axis[AXIS_Z].soft_limit_neg){
            printf("[软限位] Z轴目标位置%.2f超出软限位范围！\n", target_z);
            return 0;
        }
    }
    return 1;
}

void api_sync_planner_cursor(){
    for(int i=0;i<AXIS_NUM;i++){
        plan_cursor[i]=g_axis[i].current_cmd_pos;
    }
}

double api_get_cursor(int axis_idx){

    return plan_cursor[axis_idx];
    
}


void wait_motion_done(){
    while(g_interpolator.is_moving) osal_usleep(10000);
}
/*void api_set_zero(int axis_idx){
    g_ui_cmd.axis_idx=axis_idx;
    g_ui_cmd.cmd=CMD_SET_ZERO;
    g_ui_cmd.execute=1;
    wait_cmd_accepted();

    api_sync_planner_cursor();
}*/

void api_set_zero(int axis_idx){

    if(g_coord_mgr.current_coord==COORD_G53){
        printf("[API] G53坐标系不允许设置零点！\n");
        return;
    }

    int coord_idx=g_coord_mgr.current_coord-1; // G54对应0，G55对应1，以此类推
    for(int i=0;i<AXIS_NUM;i++){
        if(axis_idx==AXIS_ALL||axis_idx==i){
            g_coord_mgr.work_offsets[coord_idx][i]=g_coord_mgr.current_g53_pos[i];

            g_coord_mgr.current_logical_pos[i]=0.0; // 设置当前逻辑坐标为0
            plan_cursor[i]=0.0; // 同步规划器光标
        }
    }
    printf("[API] 已设置 %s 坐标系零点，当前G53位置 (%.3f, %.3f, %.3f)\n", 
            (g_coord_mgr.current_coord==COORD_G54)?"G54":"G55",
            g_coord_mgr.current_g53_pos[0], g_coord_mgr.current_g53_pos[1], g_coord_mgr.current_g53_pos[2]);
}


void api_go_zero(int axis_idx,double speed){
    double t_pos[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        t_pos[i]=plan_cursor[i];
    }
    //double tx=plan_cursor_x;
    //double ty=plan_cursor_y;
    //double tz=plan_cursor_z;

    if(axis_idx==AXIS_ALL){
        for(int i=0;i<AXIS_NUM;i++){
            t_pos[i]=0.0;
        }
    }else if(axis_idx>=0.0&&axis_idx<AXIS_NUM){
        t_pos[axis_idx]=0.0;
    }

    api_push_trajectory(t_pos,speed,DEFAULT_ACC,DEFAULT_DEC);
}



void api_move_relative(int axis_idx,double distance,double speed){
    double t_pos[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        t_pos[i]=plan_cursor[i];
    }

    if(axis_idx==AXIS_ALL){
        for(int i=0;i<AXIS_NUM;i++){
            t_pos[i]+=distance;
        }
    }else if(axis_idx>=0&&axis_idx<AXIS_NUM){
        t_pos[axis_idx]+=distance;
    }

    api_push_trajectory(t_pos,speed,DEFAULT_ACC,DEFAULT_DEC);
}


void api_push_trajectory(double target_pos[AXIS_NUM], 
                         double speed_sec_mm, double acc_sec_mm, double dec_sec_mm) 
{
    int next_head = (g_cmd_queue.head + 1) % QUEUE_SIZE;
    while (next_head == g_cmd_queue.tail) { osal_usleep(1000); } 

    
    TrajectorySegment_t *seg = &g_cmd_queue.buffer[g_cmd_queue.head];

    seg->is_ready = 0; // 标记为未准备好，等待 Planner 计算完成后设置为1

    double delta[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        delta[i]=target_pos[i]-plan_cursor[i];
    }
    double xyz_dist=0.0;
    if(AXIS_NUM>=3){
        xyz_dist=sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
    }
    double ab_dist=0.0;
    if(AXIS_NUM>=4){
        double ab_sq_sum=delta[3]*delta[3];
        if(AXIS_NUM>=5){
            ab_sq_sum+=delta[4]*delta[4];
        }
        ab_dist=sqrt(ab_sq_sum);
    }
   

    double dist=(xyz_dist>0.0001)?xyz_dist:ab_dist;
    seg->total_distance = dist;
 
    
    for(int i=0;i<AXIS_NUM;i++){
        seg->target_pos[i]=target_pos[i];
        if(dist>0.0001){
            seg->dir_vec[i]=delta[i]/dist;
        }else{
            seg->dir_vec[i]=0;
        }
        plan_cursor[i]=target_pos[i];
    }

    // 基础动力学赋值
    seg->v_target = speed_sec_mm / 1000.0;
    seg->acc = acc_sec_mm / 1000000.0;
    seg->dec = dec_sec_mm / 1000000.0;
    seg->v_start = 0.0; // 默认初始化，交给 Planner 重新计算！
    seg->v_end = 0.0;
    seg->v_max=seg->v_target; // 初始最高速等于目标速度，Planner会根据前瞻调整
    
    g_cmd_queue.head = next_head; 
    
    // 每次进队，瞬间重算整个队列的速度前瞻！
    planner_recalculate(); 
}




int is_trajectory_finished(){
    if(g_cmd_queue.head==g_cmd_queue.tail&&g_interpolator.is_moving==0){
        return 1;
    }
    return 0;
}

void api_flush_planner(){
    int curr=g_cmd_queue.tail;
    while(curr!=g_cmd_queue.head){
        g_cmd_queue.buffer[curr].is_ready=1;
        curr=(curr+1)%QUEUE_SIZE;
    }
}

void api_motion_pause(){
    g_interpolator.pause_request=1;
}

void api_motion_resume(){
    g_interpolator.pause_request=0;
}

void axis_sys_init(void)
{

    // 1. 轴数据结构初始化
    memset(g_axis,0,sizeof(g_axis));

    // 2. 插补器和命令队列初始化
    memset(&g_interpolator,0,sizeof(Interpolator_t));
    memset(&g_cmd_queue,0,sizeof(CommandQueue_t));


    // 3. 规划器光标初始化
    for(int i=0;i<AXIS_NUM;i++){
        plan_cursor[i]=0.0;
    }

    // 4. 全局状态标志初始化
    g_all_axis_op_ready=0;
    g_all_axis_reach=0;

    for(int i=0;i<AXIS_NUM;i++){
        g_axis[i].pulse_per_unit=10000.0; // 默认10000脉冲/单位
    }

    // 5.
    g_interpolator.virtual_time_ms=0.0;
    g_interpolator.time_scale=1.0;
    g_interpolator.hold_state=HOLD_NORMAL;
    g_interpolator.pause_request=0;

    printf("[系统初始化] 五轴控制系统已初始化，等待SOEM主站配置...\n");




}

/************************ 单轴SDO配置PP模式 ************************/
void axis_sdo_config_pp(int axis_idx)
{
    
    axis_sdo_config_mode(axis_idx, CSP_MODE);
    
    
    /*if (axis_idx < 0 || axis_idx >= AXIS_NUM)
    {
        printf("[SDO错误] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return;
    }
    if (g_axis[axis_idx].is_error) return; // 故障轴跳过

    uint8 pp_mode = 0x01; // CiA402 PP模式=1
    int sz = sizeof(pp_mode);
    ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, sz, &pp_mode, EC_TIMEOUTRXM);
    osal_usleep(200000);
    if (ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, sz, &pp_mode, EC_TIMEOUTRXM) == 0)
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

void axis_sdo_config_mode(int axis_idx, uint8_t mode)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM)
    {
        printf("[SDO错误] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return;
    }
    if (g_axis[axis_idx].is_error) return;

    int sz = sizeof(mode);
    // 1.写入CiA402模式值（0x6060）
    for(int s=0;s<g_axis[axis_idx].slave_count;s++){
        int wkc = ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_ids[s], 0x6060, 0x00, FALSE, sz, &mode, EC_TIMEOUTRXM);
        osal_usleep(200000); 

        //sz=sizeof(g_axis[axis_idx].pp_target_pos);
        //ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x607A, 0x00, FALSE, sz, &g_axis[axis_idx].pp_target_pos, EC_TIMEOUTRXM);
        //osal_usleep(200000);
           // 2. 读回当前模式，验证是否配置成功
    }
}

// SDO读取函数（读取指定索引/子索引的值）
uint8_t axis_sdo_read_mode(int axis_idx)
{
    uint8_t mode = 0;
    int sz = sizeof(mode);
    // 调用SOEM的SDO读函数（ecx_SDOread，与write逻辑对称）
    int wkc = ecx_SDOread(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, &sz, &mode, EC_TIMEOUTRXM);
    
    // 即使wkc=0，也返回读取到的mode（可能有效）
    return mode;
}

/************************ PDO写（控制字+目标位置） ************************/
void axis_pdo_write(int slave_id, uint16 cw, int32 pos)
{
    // 越界检查
    if (slave_id <= 0 || slave_id > ctx.slavecount) return;

    // 空指针检查+故障置位
    uint8 *out = ctx.slavelist[slave_id].outputs;
    if (!out)
    {
        printf("[PDO错误] %s PDO输出缓冲区为空！\n");
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
uint16 axis_pdo_read_sw(int slave_id)
{
    // 1. 基础校验：索引越界/轴故障/从站ID无效
    if (slave_id <= 0 || slave_id > ctx.slavecount) return 0xFFFF;

    // 2. 重试获取PDO输入缓冲区（防偶发空指针）
    uint8 *in = NULL;
    for (int retry = 0; retry < 2; retry++) {
        in = ctx.slavelist[slave_id].inputs;
        if (in) break;
        osal_usleep(1000); // 1ms重试间隔
    }

    // 3. 解析状态字（小端序：低字节在前，高字节在后）
    uint16 sw = (uint16)(in[PDO_SW_BYTE1] << 8 | in[PDO_SW_BYTE0]);
    return sw;
}

/************************ 状态位解析函数（上层调用） ************************/
// 判断某状态位是否为1
int axis_check_sw_bit(int axis_idx, uint16 bit_mask)
{
    uint16 sw = axis_pdo_read_sw(axis_idx);
    if (sw == 0xFFFF) return -1; // 读取失败
    return (sw & bit_mask) ? 1 : 0; // 1=位有效，0=位无效
}

// 打印状态字详细解析（调试用）
void axis_print_sw_detail(int axis_idx)
{
    uint16 sw = axis_pdo_read_sw(axis_idx);
    if (sw == 0xFFFF) {
        printf("[状态字解析] %s 读取失败！\n", g_axis[axis_idx].axis_name);
        return;
    }

    printf("[状态字解析] %s 完整值：0x%04X\n", g_axis[axis_idx].axis_name, sw);
    printf("  - 准备功能启动：%s\n", (sw & SW_READY_FUNC_START) ? "是" : "否");
    printf("  - 伺服准备完成：%s\n", (sw & SW_SERVO_READY) ? "是" : "否");
    printf("  - 伺服使能：%s\n", (sw & SW_SERVO_ENABLE) ? "是" : "否");
    printf("  - 异常信号：%s\n", (sw & SW_ERROR) ? "是" : "否");
    printf("  - 入力侧供电：%s\n", (sw & SW_MAIN_POWER_ON) ? "是" : "否");
    printf("  - 紧急停止：%s\n", (sw & SW_EMERGENCY_STOP) ? "是" : "否");
    printf("  - 警告信号：%s\n", (sw & SW_WARNING) ? "是" : "否");
    printf("  - 远程控制：%s\n", (sw & SW_REMOTE_CTRL) ? "是" : "否");
    printf("  - 目标到达：%s\n", (sw & SW_TARGET_REACH) ? "是" : "否");
}


/************************ 配置CSP模式所有参数 ************************/
void axis_config_csp_params(int axis_idx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    
    int slave_id = g_axis[axis_idx].slave_id;
    printf("\n========== 配置 %s CSP参数（修正版） ==========\n", g_axis[axis_idx].axis_name);
    
    // 1. 最大轮廓速度 (0x607F) - 单位: pulse/s
    // 使用驱动器实际支持的值（从读回值看，60000是有效的）
    uint32_t profile_velocity = 60000;  // 60k pulse/s = 60 pulse/ms
    int wkc = ecx_SDOwrite(&ctx, slave_id, 0x607F, 0x00, FALSE, 4, &profile_velocity, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    // 读回验证
    uint32_t read_vel = 0;
    int sz = sizeof(read_vel);
    ecx_SDOread(&ctx, slave_id, 0x607F, 0x00, FALSE, &sz, &read_vel, EC_TIMEOUTRXM);
    printf("0x607F 最大轮廓速度: %d pulse/s\n", read_vel);
    
    for(int i=0;i<AXIS_NUM;i++){
        int32_t actual_pos = 0;
        sz = sizeof(actual_pos);
        ecx_SDOread(&ctx, g_axis[i].slave_id, 0x6064, 0x00, FALSE, &sz, &actual_pos, EC_TIMEOUTRXM);
        g_axis[i].target_pos=actual_pos;
    }
    /*// 2. 加速度 (0x6083) - 使用驱动器支持的值
    uint32_t acceleration = 10000;  // 从读回值看，19264附近是有效的
    ecx_SDOwrite(&ctx, slave_id, 0x6083, 0x00, FALSE, 4, &acceleration, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    uint32_t read_acc = 0;
    ecx_SDOread(&ctx, slave_id, 0x6083, 0x00, FALSE, &sz, &read_acc, EC_TIMEOUTRXM);
    printf("0x6083 加速度: %d pulse/s²\n", read_acc);
    
    // 3. 减速度 (0x6084) - 与加速度相同
    ecx_SDOwrite(&ctx, slave_id, 0x6084, 0x00, FALSE, 4, &acceleration, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    uint32_t read_dec = 0;
    ecx_SDOread(&ctx, slave_id, 0x6084, 0x00, FALSE, &sz, &read_dec, EC_TIMEOUTRXM);
    printf("0x6084 减速度: %d pulse/s²\n", read_dec);
     */ 
    // 4. 软件限位保持不变
    int32_t min_pos = -1000000;
    int32_t max_pos = 1000000;
    ecx_SDOwrite(&ctx, slave_id, 0x607D, 0x01, FALSE, 4, &min_pos, EC_TIMEOUTRXM);
    osal_usleep(30000);
    ecx_SDOwrite(&ctx, slave_id, 0x607D, 0x02, FALSE, 4, &max_pos, EC_TIMEOUTRXM);
    osal_usleep(30000);
    
    printf("========== %s CSP参数配置完成 ==========\n\n", g_axis[axis_idx].axis_name);
}


/************************ 读取CSP运行状态 ************************/
void axis_read_csp_status(int axis_idx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    
    int slave_id = g_axis[axis_idx].slave_id;
    
    // 1. 状态字
    uint16_t sw = axis_pdo_read_sw(axis_idx);
    
    // 2. 当前模式 (0x6061)
    uint8_t current_mode = 0;
    int sz = sizeof(current_mode);
    ecx_SDOread(&ctx, slave_id, 0x6061, 0x00, FALSE, &sz, &current_mode, EC_TIMEOUTRXM);
    
    // 3. 实际位置 (0x6064)
    int32_t actual_pos = 0;
    sz = sizeof(actual_pos);
    ecx_SDOread(&ctx, slave_id, 0x6064, 0x00, FALSE, &sz, &actual_pos, EC_TIMEOUTRXM);
    
    // 4. 跟随误差 (0x60F4)
    int32_t following_error = 0;
    sz = sizeof(following_error);
    ecx_SDOread(&ctx, slave_id, 0x60F4, 0x00, FALSE, &sz, &following_error, EC_TIMEOUTRXM);
    
    // 5. 速度实际值 (0x606C)
    int32_t actual_vel = 0;
    sz = sizeof(actual_vel);
    ecx_SDOread(&ctx, slave_id, 0x606C, 0x00, FALSE, &sz, &actual_vel, EC_TIMEOUTRXM);
    
    printf("\n========== %s 运行状态 ==========\n", g_axis[axis_idx].axis_name);
    printf("状态字: 0x%04X (bit3=%d, bit12=%d)\n", sw, (sw>>3)&1, (sw>>12)&1);
    printf("当前模式 (0x6061): %d %s\n", current_mode, 
           (current_mode == 8) ? "(CSP)" : "(其他)");
    printf("目标位置: %d\n", g_axis[axis_idx].target_pos);
    printf("实际位置 (0x6064): %d\n", actual_pos);
    printf("跟随误差 (0x60F4): %d\n", following_error);
    printf("实际速度 (0x606C): %d pulse/s\n", actual_vel);
    
    if(actual_pos != g_axis[axis_idx].target_pos) {
        printf("⚠️ 实际位置与目标位置不符！差值: %d\n", 
               g_axis[axis_idx].target_pos - actual_pos);
    }
    
    if(following_error > 1000) {
        printf("⚠️ 跟随误差过大！\n");
    }
    
    printf("=====================================\n");
}

void check_pdo_mapping(int axis_idx)
{
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("\n========== %s PDO映射检查 ==========\n", g_axis[axis_idx].axis_name);
    
    // 检查RxPDO分配 (0x1C12)
    uint8_t sub_count = 0;
    int sz = 1;
    ecx_SDOread(&ctx, slave_id, 0x1C12, 0x00, FALSE, &sz, &sub_count, EC_TIMEOUTRXM);
    printf("RxPDO分配 (0x1C12) 子索引数: %d\n", sub_count);
    
    for(int i = 1; i <= sub_count && i <= 10; i++) {
        uint16_t pdo_num = 0;
        sz = 2;
        ecx_SDOread(&ctx, slave_id, 0x1C12, i, FALSE, &sz, &pdo_num, EC_TIMEOUTRXM);
        printf("  RxPDO[%d] = 0x%04X\n", i, pdo_num);
        
        // 打印RxPDO详细映射
        if(pdo_num >= 0x1600 && pdo_num <= 0x1603) {
            uint8_t map_sub = 0;
            sz = 1;
            ecx_SDOread(&ctx, slave_id, pdo_num, 0x00, FALSE, &sz, &map_sub, EC_TIMEOUTRXM);
            printf("    RxPDO 0x%04X 映射对象数: %d\n", pdo_num, map_sub);
            
            for(int j = 1; j <= map_sub; j++) {
                uint32_t mapping = 0;
                sz = 4;
                ecx_SDOread(&ctx, slave_id, pdo_num, j, FALSE, &sz, &mapping, EC_TIMEOUTRXM);
                uint16_t index = (mapping >> 16) & 0xFFFF;
                uint8_t sub = (mapping >> 8) & 0xFF;
                uint8_t bits = mapping & 0xFF;
                printf("      0x%04X:%02X (%d bits)\n", index, sub, bits);
            }
        }
    }
    
    // 检查TxPDO分配 (0x1C13) - 关键修改：打印详细映射
    sz = 1;
    ecx_SDOread(&ctx, slave_id, 0x1C13, 0x00, FALSE, &sz, &sub_count, EC_TIMEOUTRXM);
    printf("\nTxPDO分配 (0x1C13) 子索引数: %d\n", sub_count);
    
    for(int i = 1; i <= sub_count && i <= 10; i++) {
        uint16_t pdo_num = 0;
        sz = 2;
        ecx_SDOread(&ctx, slave_id, 0x1C13, i, FALSE, &sz, &pdo_num, EC_TIMEOUTRXM);
        printf("  TxPDO[%d] = 0x%04X\n", i, pdo_num);
        
        // ========== 关键修改：打印TxPDO详细映射 ==========
        if(pdo_num >= 0x1A00 && pdo_num <= 0x1A03) {
            uint8_t map_sub = 0;
            sz = 1;
            ecx_SDOread(&ctx, slave_id, pdo_num, 0x00, FALSE, &sz, &map_sub, EC_TIMEOUTRXM);
            printf("    TxPDO 0x%04X 映射对象数: %d\n", pdo_num, map_sub);
            
            for(int j = 1; j <= map_sub; j++) {
                uint32_t mapping = 0;
                sz = 4;
                ecx_SDOread(&ctx, slave_id, pdo_num, j, FALSE, &sz, &mapping, EC_TIMEOUTRXM);
                uint16_t index = (mapping >> 16) & 0xFFFF;
                uint8_t sub = (mapping >> 8) & 0xFF;
                uint8_t bits = mapping & 0xFF;
                printf("      0x%04X:%02X (%d bits)\n", index, sub, bits);
                
                // 检查是否包含实际位置
                if(index == 0x6064) {
                    printf("      ✅ 包含实际位置 (0x6064)\n");
                }
            }
        }
    }
    
    printf("=====================================\n");
}

void read_error_history(int axis_idx)
{
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("\n========== %s 故障历史 ==========\n", g_axis[axis_idx].axis_name);
    
    // 读取故障码数量
    uint8_t err_count = 0;
    int sz = sizeof(err_count);
    ecx_SDOread(&ctx, slave_id, 0x1003, 0x00, FALSE, &sz, &err_count, EC_TIMEOUTRXM);
    printf("故障记录数: %d\n", err_count);
    
    // 读取每个故障码
    for(int i = 1; i <= err_count && i <= 8; i++) {
        uint32_t err_code = 0;
        sz = sizeof(err_code);
        ecx_SDOread(&ctx, slave_id, 0x1003, i, FALSE, &sz, &err_code, EC_TIMEOUTRXM);
        printf("故障[%d]: 0x%08X\n", i, err_code);
        
        // 常见故障码解释
        switch(err_code & 0xFFFF) {
            case 0x2310:
                printf("  位置跟随误差过大\n");
                break;
            case 0x2320:
                printf("  速度过快\n");
                break;
            case 0x5110:
                printf("  参数设置错误\n");
                break;
            case 0x7380:
                printf("  紧急停止\n");
                break;
            default:
                printf("  请查阅台达B3手册\n");
        }
    }
    
    // 清除故障历史（谨慎使用）
    // uint8_t clear = 0;
    // ecx_SDOwrite(&ctx, slave_id, 0x1003, 0x00, FALSE, 1, &clear, EC_TIMEOUTRXM);
    
    printf("====================================\n");
}

/*********************** 写控制字（使能过程用） ************************/
void axis_pdo_write_cw_only(int axis_idx, uint16 cw)
{
    // 越界检查
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    if (g_axis[axis_idx].is_error) return;

    uint8 *out = ctx.slavelist[g_axis[axis_idx].slave_id].outputs;
    if (!out)
    {
        g_axis[axis_idx].is_error = 1;
        printf("[PDO错误] %s PDO输出缓冲区为空！\n", g_axis[axis_idx].axis_name);
        return;
    }

    // 写入控制字
    out[PDO_CW_BYTE0] = cw & 0xFF;
    out[PDO_CW_BYTE1] = (cw >> 8) & 0xFF;
    
  
}

void diagnose_sync_failure(int axis_idx)
{
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("\n========== %s 同步诊断 ==========\n", g_axis[axis_idx].axis_name);
    
    // 1. 检查DC同步状态 (0x1C32:08)
    uint16_t sync_error = 0;
    int sz = 2;
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x08, FALSE, &sz, &sync_error, EC_TIMEOUTRXM);
    printf("0x1C32:08 (同步错误) = 0x%04X\n", sync_error);
    
    // 2. 检查周期过小/过大计数
    uint32_t cycle_small = 0, cycle_large = 0;
    sz = 4;
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x0A, FALSE, &sz, &cycle_small, EC_TIMEOUTRXM);
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x0B, FALSE, &sz, &cycle_large, EC_TIMEOUTRXM);
    printf("周期过小计数: %d\n", cycle_small);
    printf("周期过大计数: %d\n", cycle_large);
    
    // 3. 检查当前同步模式 (0x1C32:01)
    uint16_t sync_mode = 0;
    sz = 2;
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x01, FALSE, &sz, &sync_mode, EC_TIMEOUTRXM);
    printf("当前同步模式: %d (2=DC模式)\n", sync_mode);
    
    // 4. 检查DC激活状态
    printf("DCactive: %d\n", ctx.slavelist[slave_id].DCactive);
    printf("DCcycle: %d ns\n", ctx.slavelist[slave_id].DCcycle);
    
    // 5. 读取警告码 (0x1003可能记录警告)
    uint8_t err_count = 0;
    sz = 1;
    ecx_SDOread(&ctx, slave_id, 0x1003, 0x00, FALSE, &sz, &err_count, EC_TIMEOUTRXM);
    printf("故障记录数: %d\n", err_count);
    for(int i = 1; i <= err_count; i++) {
        uint32_t err_code = 0;
        sz = 4;
        ecx_SDOread(&ctx, slave_id, 0x1003, i, FALSE, &sz, &err_code, EC_TIMEOUTRXM);
        printf("  故障[%d]: 0x%08X\n", i, err_code);
    }
    
    printf("====================================\n");
}



/************************ 从PDO读取实际位置 ************************/
int32 axis_pdo_read_pos(int slave_id)
{
    if (slave_id <=0 || slave_id > ctx.slavecount) return 0;
    
    uint8 *in = ctx.slavelist[slave_id].inputs;
    if (!in) return 0;
    
    // TxPDO 0x1A01 映射：偏移0-1状态字，偏移2-5实际位置
    int32_t actual_pos = (int32_t)(
        in[2] | 
        (in[3] << 8) | 
        (in[4] << 16) | 
        (in[5] << 24)
    );
    
    return actual_pos;
}

/************************ 执行原点复归 (Homing) ************************/
void axis_homing(int axis_idx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("[Homing] %s 开始原点复归...\n", g_axis[axis_idx].axis_name);
    
    // 1. 切换到原点复归模式 (0x6060 = 6)
    uint8_t homing_mode = 6;
    ecx_SDOwrite(&ctx, slave_id, 0x6060, 0x00, FALSE, 1, &homing_mode, EC_TIMEOUTRXM);
    osal_usleep(100000);
    
    // 2. 设置原点复归方法 (0x6098 = 35) —— 将当前位置设为原点
    int8_t homing_method = 35;
    ecx_SDOwrite(&ctx, slave_id, 0x6098, 0x00, FALSE, 1, &homing_method, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    // 3. 触发原点复归（控制字 bit4 = 1）
    // 注意：需要先使能，然后发送带触发位的控制字
    // 假设此时伺服已在使能状态（状态字0x0237）
    axis_pdo_write_cw_only(axis_idx, 0x001F);  // 0x001F = 使能 + 触发位
    ecx_send_processdata(&ctx);
    
    // 4. 等待复归完成（状态字 bit12 可能变为1，或等待一段时间）
    int timeout = 100; // 100ms * 100 = 10s
    uint16_t sw = 0;
    do {
        osal_usleep(100000);
        sw = axis_pdo_read_sw(axis_idx);
        timeout--;
        if (timeout == 0) {
            printf("[Homing] %s 超时！\n", g_axis[axis_idx].axis_name);
            break;
        }
    } while (!(sw & 0x1000)); // 假设 bit12 表示复归完成（需查手册）
    
    printf("[Homing] %s 完成，状态字=0x%04X\n", g_axis[axis_idx].axis_name, sw);
    
    // 5. 切换回CSP模式
    uint8_t csp_mode = 8;
    ecx_SDOwrite(&ctx, slave_id, 0x6060, 0x00, FALSE, 1, &csp_mode, EC_TIMEOUTRXM);
    osal_usleep(100000);
}
