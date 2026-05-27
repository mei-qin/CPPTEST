#include "ecat_core.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/************************ 全局变量定义（SOEM+实时控制相关） ************************/
// SOEM核心变量
ecx_contextt ctx;
uint8 IOmap[4096];
OSAL_THREAD_HANDLE thread_rt;
OSAL_THREAD_HANDLE thread_chk;
OSAL_THREAD_HANDLE thread_parser;
int expectedWKC;
int wkc;
int mappingdone = 0, dorun = 0, inOP = 0;
int dowkccheck = 0, currentgroup = 0;
int g_all_axis_enabled = 0;

// 实时控制变量
int cycle = 0;
int g_csp_ready = 0;
int64 cycletime = CYCLE_TIME_NS;
int64 syncoffset = 500000;
int64 timeerror;
float pgain = 0.01f;
float igain = 0.00002f;
extern struct timespec ts;




/************************ 时间工具：纳秒级时间累加 ************************/
void add_time_ns(ec_timet *ts, int64 addtime)
{
    ec_timet addts;
    addts.tv_nsec = addtime % NSEC_PER_SEC;
    addts.tv_sec  = (addtime - addts.tv_nsec) / NSEC_PER_SEC;
    osal_timespecadd(ts, &addts, ts);
}


/**
 * 同步函数：根据从站DC时间调整主站下一个周期的唤醒时刻。
 * @param reftime    从站当前DC时间（单位 ns），从 ctx.DCtime 获取
 * @param cycletime  期望的通信周期（单位 ns），如 CYCLE_TIME_NS
 * @param offsettime 输出：计算出的补偿值，用于调整下一个周期的等待时间
 * @param ts         下一个周期的绝对目标时间（输入/输出），调用者需维护
 */
void ec_sync(int64 reftime,int64 cycletime,int64 *offsettime){
    static int64 integral=0;
    int64 delta;
    delta=(reftime-syncoffset)%cycletime;
    if(delta>(cycletime/2)){
        delta=delta-cycletime;
    }
    timeerror=-delta;
    integral+=timeerror;
    *offsettime=(int64)((timeerror*pgain)+(integral*igain));
}

/************************ 实时控制线程（核心！1ms周期，五轴同周期控制） ************************/
OSAL_THREAD_FUNC_RT ecat_thread_rt(void *arg)
{
    ec_timet ts;
    int ht;
    static int64_t toff = 0;
    //static int64_t motion_cycle = 0;
    uint16 sw = 0; // 临时缓存状态字，减少重复读取

    dorun=0;
    while (!mappingdone) osal_usleep(100);
    osal_get_monotonic_time(&ts);

    // 在osal_get_monotonic_time(&ts); 后新增：
    // 1. 设置线程优先级（Linux实时调度）
    // 在实时线程开头，设置定时器精度（可选）
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);  // 假设绑定到 CPU 3，根据实际系统调整
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    struct sched_param sp = {.sched_priority = 99};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    struct sched_param param;
    param.sched_priority = 99; // 最高实时优先级
    sched_setscheduler(0, SCHED_FIFO, &param);
    printf("[实时线程] 已设置FIFO调度，优先级99\n");

    osal_get_monotonic_time(&ts); // 获取当前时间，准备下一周期
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000; // 校准到毫秒整，消除启动延时
    ecx_send_processdata(&ctx);
    printf("[实时线程] 初始化完成，等待dorun启动\n");

    while (1)
    {
        // 1ms实时定时
        add_time_ns(&ts, cycletime + toff);


        osal_monotonic_sleep(&ts);
        

        if(dorun>0){
            cycle++;
            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

            static int is_first_run=1;
            if(is_first_run&&wkc>0){
                for(int i=0;i<AXIS_NUM;i++){
                    for(int s=0;s<g_axis[i].slave_count;s++){
                        int slave_id=g_axis[i].slave_ids[s];
                        g_axis[i].home_offset[s]=axis_pdo_read_pos(slave_id);
                    }
                }

            }
            is_first_run=0;
            
    
            // DC时钟同步
            if (ctx.slavelist[0].hasdc && (wkc > 0))
                ec_sync(ctx.DCtime, cycletime, &toff);
            
            //g_all_axis_op_ready=1;
            if(g_all_axis_op_ready){
                for(int i=0;i<AXIS_NUM;i++){
                    if(g_axis[i].slave_count>=2&&g_axis[i].enable_sync_alarm){
                        int slave_1=g_axis[i].slave_ids[0];
                        int slave_2=g_axis[i].slave_ids[1];

                        int32_t act_1=axis_pdo_read_pos(slave_1)-g_axis[i].home_offset[0];
                        int32_t act_2=axis_pdo_read_pos(slave_2)-g_axis[i].home_offset[1];

                        int32_t diff=abs(act_1-act_2);
                        if(diff>g_axis[i].sync_max_err_pulse){
                           printf("同步异常！轴%d 从站%d与%d位置差%d超过设定的最大误差%d\n", i, slave_1, slave_2, diff, g_axis[i].sync_max_err_pulse);
                           dorun=0;
                           continue;
                        }

                        if(diff>g_axis[i].sync_tolerance_pulse){
                            g_axis[i]._current_sync_timer++; //1ms周期,累积同步误差持续时间
                            if(g_axis[i]._current_sync_timer>g_axis[i].sync_err_time_ms){
                                printf("同步报警！轴%d 从站%d与%d位置差%d超过设定的容差%d持续时间%dms超过设定的时间%dms\n", i, slave_1, slave_2, diff, g_axis[i].sync_tolerance_pulse, g_axis[i]._current_sync_timer, g_axis[i].sync_err_time_ms);
                                dorun=0;
                            }
                        }else{
                            g_axis[i]._current_sync_timer=0; //误差恢复正常，重置计时器
                        }
                    }
                }
            }

            if(g_all_axis_op_ready&&g_interpolator.is_moving==0&&g_cmd_queue.head==g_cmd_queue.tail)
            {
                if(g_ui_cmd.execute==1)
                {
                    if(g_ui_cmd.cmd==CMD_SET_ZERO){
                        for(int i=0;i<AXIS_NUM;i++){
                            if(g_ui_cmd.axis_idx==AXIS_ALL||g_ui_cmd.axis_idx==i){
                                int32_t current_logical_pulse=(int32_t)round(g_axis[i].current_cmd_pos*g_axis[i].pulse_per_unit);

                                for(int s=0;s<g_axis[i].slave_count;s++){
                                    g_axis[i].home_offset[s]=current_logical_pulse+g_axis[i].home_offset[s];
                                }
                                g_axis[i].current_cmd_pos=0;


                                g_interpolator.current_pos[i]=0.0;
                               
                            }
                        }

                        g_ui_cmd.execute=0;

                    }
                }
            }


            // 5.
            if(g_all_axis_op_ready){


                if(g_interpolator.is_moving==0&&g_interpolator.is_waiting_mcode==0&&g_cmd_queue.head!=g_cmd_queue.tail){

                    TrajectorySegment_t seg=g_cmd_queue.buffer[g_cmd_queue.tail];

                    if(g_cmd_queue.buffer[g_cmd_queue.tail].is_ready==1){
                        g_cmd_queue.tail=(g_cmd_queue.tail+1)%QUEUE_SIZE;

                        // M代码段：置屏障，不加载到插补器
                        if(seg.cmd_type==CMD_TYPE_MCODE){
                            g_interpolator.is_waiting_mcode=1;
                            g_interpolator.mcode_wait_timer=0;
                            g_interpolator.current_mcode=seg.m_code;
                            // 注意：实时线程禁止printf，日志由非实时线程或环形缓冲处理
                            continue;
                        }

                        for(int j=0;j<AXIS_NUM;j++){
                            g_interpolator.start_pos[j]=g_interpolator.current_pos[j];
                            g_interpolator.target_pos[j]=seg.target_pos[j];
                            g_interpolator.dir_vec[j]=seg.dir_vec[j];
                        }
                   
                        g_interpolator.total_distance=seg.total_distance;

                        g_interpolator.t_acc=seg.t_acc;
                        g_interpolator.t_dec=seg.t_dec;
                        g_interpolator.t_cru=seg.t_cru;
                        g_interpolator.v_target=seg.v_target;
                        g_interpolator.acc=seg.acc;
                        g_interpolator.dec=seg.dec;
                        g_interpolator.total_time_ms=seg.t_total;
                        g_interpolator.v_start=seg.v_start;
                        g_interpolator.v_end=seg.v_end;
                        //g_interpolator.current_time_ms=0;
                        g_interpolator.virtual_time_ms=0.0;

                        if(g_interpolator.total_time_ms>0){
                            g_interpolator.is_moving=1;
                        }else{
                            printf("invalid trajectory segment: total_time_ms=%d\n",g_interpolator.total_time_ms);
                        }
                    }
                }


                // 非阻塞M代码等待逻辑（每ms周期递增计数器，无sleep）
                if(g_interpolator.is_waiting_mcode){
                    g_interpolator.mcode_wait_timer++;
                    int wait_target_ms;
                    switch(g_interpolator.current_mcode){
                        case 3:  wait_target_ms=2000; break; // M03 主轴正转
                        case 5:  wait_target_ms=1000; break; // M05 主轴停止
                        default: wait_target_ms=1000; break; // 其他M代码默认1000ms
                    }
                    // 正常到达目标延时，或绝对超时兜底防死锁
                    if(g_interpolator.mcode_wait_timer >= wait_target_ms ||
                       g_interpolator.mcode_wait_timer >= MCODE_WAIT_TIMEOUT_MS){
                        g_interpolator.is_waiting_mcode=0;
                        g_interpolator.mcode_wait_timer=0;
                    }
                }

                if(g_interpolator.is_moving){


                    // ==========================================
                    // 1. 虚拟时间轴控制器 (Feedhold 核心)
                    // ==========================================
                    // 状态机切换
                    if (g_interpolator.pause_request && g_interpolator.hold_state == HOLD_NORMAL) {
                        g_interpolator.hold_state = HOLD_BRAKING;
                    } else if (!g_interpolator.pause_request && g_interpolator.hold_state == HOLD_PAUSED) {
                        g_interpolator.hold_state = HOLD_RESUMING;
                    }

                    // 设定平滑刹车/恢复的加速度 (例如每毫秒降低 0.005，意味着 200ms 内平滑停稳)
                    // 这里的数值可以根据机床实际重量调整，越小刹车越柔和，越大刹车越急
                    const double TIME_DEC_STEP = 0.005; 

                    if (g_interpolator.hold_state == HOLD_BRAKING) {
                        g_interpolator.time_scale -= TIME_DEC_STEP;
                        if (g_interpolator.time_scale <= 0.0) {
                            g_interpolator.time_scale = 0.0;
                            g_interpolator.hold_state = HOLD_PAUSED;
                        }
                    } else if (g_interpolator.hold_state == HOLD_RESUMING) {
                        g_interpolator.time_scale += TIME_DEC_STEP;
                        if (g_interpolator.time_scale >= 1.0) {
                            g_interpolator.time_scale = 1.0;
                            g_interpolator.hold_state = HOLD_NORMAL;
                        }
                    }

                    g_interpolator.virtual_time_ms+=g_interpolator.time_scale;

                    // 获取当前插补瞬间的“虚拟绝对时间”
                    double t_val = g_interpolator.virtual_time_ms;
                    
                    // ==========================================
                    // 2. 绝对数学解析器 (保持不变，直接接管虚拟时间)
                    // ==========================================
                    double s = 0.0;
                    double t1_val = (double)g_interpolator.t_acc;
                    double t2_val = (double)(g_interpolator.t_acc + g_interpolator.t_cru);

                    double s_acc_phase = 0.5 * g_interpolator.acc * t1_val * t1_val + g_interpolator.v_start * t1_val;
                    double v_acc_phase = g_interpolator.acc * t1_val + g_interpolator.v_start;
                    double s_cru_phase = s_acc_phase + v_acc_phase * g_interpolator.t_cru;

                    if (t_val <= t1_val) {
                        s = 0.5 * g_interpolator.acc * t_val * t_val + g_interpolator.v_start * t_val;
                    } 
                    else if (t_val <= t2_val) {
                        double dt = t_val - t1_val;
                        s = s_acc_phase + v_acc_phase * dt;
                    } 
                    else if (t_val <= g_interpolator.total_time_ms) {
                        double dt = t_val - t2_val;
                        s = s_cru_phase + v_acc_phase * dt - 0.5 * g_interpolator.dec * dt * dt;
                    } 
                    else {
                        s = g_interpolator.total_distance;
                        g_interpolator.is_moving = 0; // 一段轨迹彻底走完
                    }
           

                    double ratio=0;
                    if(g_interpolator.total_distance>0.000001){
                        ratio=s/g_interpolator.total_distance;
                    }

                    if(ratio>1.0) ratio=1.0;
                    if(ratio<0.0) ratio=0.0;

                    for(int j=0;j<AXIS_NUM;j++){
                        g_interpolator.current_pos[j]=g_interpolator.start_pos[j]+(g_interpolator.target_pos[j]-g_interpolator.start_pos[j])*ratio;
                    }

                }
                for(int j=0;j<AXIS_NUM;j++){
                    g_coord_mgr.current_g53_pos[j]=g_interpolator.current_pos[j];


                    if(g_coord_mgr.current_coord==COORD_G53){
                        g_coord_mgr.current_logical_pos[j]=g_coord_mgr.current_g53_pos[j];
                    }else{
                        int idx=g_coord_mgr.current_coord-1;
                        g_coord_mgr.current_logical_pos[j]=g_coord_mgr.current_g53_pos[j]-g_coord_mgr.work_offsets[idx][j];
                    }
                }
            }



            int all_ready_flag=1;

            for(int i=0;i<AXIS_NUM;i++){
                int primary_slave=g_axis[i].slave_ids[0];
                uint16_t sw=axis_pdo_read_sw(primary_slave);
                g_axis[i].cia_step_delay++;

                int32_t output_cw=CW_ENABLE_OP;
                //int32_t output_logical_pos=0;

                switch(g_axis[i].cia_step){
                    case 0:
                    output_cw=CW_SHUTDOWN;
                    if(g_axis[i].cia_step_delay==1){
                        int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                        g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;
                    }
                    if(g_axis[i].cia_step_delay>50&&((sw&SW_MASK)==SW_SHUTDOWN_RDY)){
                        g_axis[i].cia_step++;
                        g_axis[i].cia_step_delay=0;
                    }
                    all_ready_flag=0;
                    break;

                    case 1:
                    output_cw=CW_SWITCH_ON;
                    if(g_axis[i].cia_step_delay==1){
                        int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                        g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;
                    }
                    if(g_axis[i].cia_step_delay>50&&((sw&SW_MASK)==SW_SWITCHED_ON)){
                        if(llabs(timeerror)<80000){
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay=0;
                        }
                    }
                    all_ready_flag=0;
                    break;

                    case 2:
                    output_cw=CW_ENABLE_OP;
                    if(g_axis[i].cia_step_delay==1){
                        int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                        g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;
                    }
                    if((sw&SW_MASK)==SW_OP_ENABLED){
                        g_axis[i].cia_step_delay++;
                        if(g_axis[i].cia_step_delay>50){
                            g_axis[i].cia_step_delay=0;
                            g_axis[i].cia_step++;

                            int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                            g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;

                            g_interpolator.current_pos[i]=g_axis[i].current_cmd_pos;
                            //if(i==AXIS_X) g_interpolator.current_x=g_axis[i].current_cmd_pos;
                            //else if(i==AXIS_Y) g_interpolator.current_y=g_axis[i].current_cmd_pos;
                            //else if(i==AXIS_Z) g_interpolator.current_z=g_axis[i].current_cmd_pos;

                            api_sync_planner_cursor();
                        }

                    }

                    all_ready_flag=0;
                    break;

                    case 3:
                    output_cw=CW_ENABLE_OP;

                    g_axis[i].current_cmd_pos=g_interpolator.current_pos[i];
                    //if(i==AXIS_X) g_axis[i].current_cmd_pos=g_interpolator.current_x;
                    //else if(i==AXIS_Y) g_axis[i].current_cmd_pos=g_interpolator.current_y;
                    //else if(i==AXIS_Z) g_axis[i].current_cmd_pos=g_interpolator.current_z;
                
                    break;
                }


                for(int s=0;s<g_axis[i].slave_count;s++){
                    int slave_id=g_axis[i].slave_ids[s];
                    int32_t logical_pulse=(int32_t)round(g_axis[i].current_cmd_pos*g_axis[i].pulse_per_unit);
                    int32_t phys_pos_to_send=logical_pulse+g_axis[i].home_offset[s];
                    axis_pdo_write(slave_id,output_cw,phys_pos_to_send);
                }
            }

            g_all_axis_op_ready=all_ready_flag;
                  

    

            // SOEM底层：处理邮箱+批量发送所有轴PDO数据
            ecx_mbxhandler(&ctx, 0, 4);
            ecx_send_processdata(&ctx);

        
            
        }else{
            // 系统停止/异常分支：重置M代码屏障防止标志位残留
            g_interpolator.is_waiting_mcode=0;
            g_interpolator.mcode_wait_timer=0;
            for(int i=0;i<AXIS_NUM;i++){
               for(int s=0;s<g_axis[i].slave_count;s++){
                int slave_id=g_axis[i].slave_ids[s];
                int32_t act=axis_pdo_read_pos(slave_id);
                axis_pdo_write(slave_id,CW_SHUTDOWN,act);
               }
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
                for(int s=0;s<g_axis[i].slave_count;s++){
                    slaveix = g_axis[i].slave_ids[s];
                    ec_slavet *slave = &ctx.slavelist[slaveix];
                    if (slave->state != EC_STATE_OPERATIONAL)
                    {
                       ctx.grouplist[currentgroup].docheckstate = TRUE;
                       g_axis[i].is_error = 1;
                       printf("[故障检测] %s 非OP状态！当前状态码：%d\n", g_axis[i].axis_name, slave->state);
                    }
                }
            }
            dowkccheck = 0;
        }
        osal_usleep(10000); // 10ms检测一次，非实时，降低CPU占用
    }
    return ;
}

/************************ ECAT主站启动封装（所有初始化） ************************/
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

    for(int i=1;i<=ctx.slavecount;i++){
        ecx_dcsync0(&ctx,i,TRUE,cycletime,0);
    }

    printf("[ECAT] trun to PRE_OP!\n");
    for(int i=0;i<AXIS_NUM;i++){
        for(int s=0;s<g_axis[i].slave_count;s++){
          uint16_t slave=g_axis[i].slave_ids[s];
          if(slave==0) continue;

          ctx.slavelist[slave].state=EC_STATE_PRE_OP;
          ecx_writestate(&ctx,slave);
          ecx_statecheck(&ctx,slave,EC_STATE_PRE_OP,EC_TIMEOUTSTATE);


          uint16_t sync_mode=2;
          int sz=sizeof(sync_mode);
          int r1=ecx_SDOwrite(&ctx,slave,0x1C32,0x01,FALSE,sz,&sync_mode,EC_TIMEOUTRXM);
          int r2=ecx_SDOwrite(&ctx,slave,0x1C33,0x01,FALSE,sz,&sync_mode,EC_TIMEOUTRXM);
          if(r1>0&&r2>0){
              printf("success to 0x1C32=2\n",g_axis[i].axis_name);
          }else{
              printf("fail to 0x1C32=2\n",g_axis[i].axis_name);
          }
          ctx.slavelist[slave].state=EC_STATE_SAFE_OP;
          ecx_writestate(&ctx,slave);
          ecx_statecheck(&ctx,slave,EC_STATE_SAFE_OP,EC_TIMEOUTSTATE);
        }

    }

    for(int i=0;i<AXIS_NUM;i++){
        for(int s=0;s<g_axis[i].slave_count;s++){
          uint16_t slave=g_axis[i].slave_ids[s];
          if(slave==0) continue;

          uint8_t time_value=1;
          int8_t time_index=-3;
          int r1=ecx_SDOwrite(&ctx,slave,0x60C2,0x01,FALSE,1,&time_value,EC_TIMEOUTRXM);
          int r2=ecx_SDOwrite(&ctx,slave,0x60C2,0x02,FALSE,1,&time_index,EC_TIMEOUTRXM);

          if(r1>0&&r2>0){
              printf("success 0x60C2\n",g_axis[i].axis_name);
          }else{
              printf("fail to 0x60C2\n",g_axis[i].axis_name);
          }
        }
    
    }



    expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;
    printf("[ECAT] PDO映射+DC配置完成，期望WKC值：%d\n", expectedWKC);


    for (int i = 0; i < AXIS_NUM; i++)
    {

        for(int s=0;s<g_axis[i].slave_count;s++){
            uint16_t slave=g_axis[i].slave_ids[s];
        // 7. 全局切换到OP状态（
            ctx.slavelist[slave].state = EC_STATE_SAFE_OP;
            ecx_writestate(&ctx, slave);
            if (ecx_statecheck(&ctx, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP)
            {
               printf("[ECAT错误] %s 无法进入SAFE_OP状态！\n", g_axis[i].axis_name);
               ecx_close(&ctx);
               exit(-1);
            }
        }
    
    }
    printf("[ECAT] 所有轴已进入SAFE_OP状态，开始SDO配置\n");
    
    // 6. 所有轴SDO配置模式
    for (int i = 0; i < AXIS_NUM; i++)
    {
        axis_sdo_config_pp(i);
    }


    // 置映射完成标志，启动实时线程
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
