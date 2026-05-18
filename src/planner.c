#include "planner.h"

// 辅助函数：计算两个线段的夹角，返回安全过弯速度
// 夹角越小（越平滑），过弯速度越高；夹角接近 90 度，必须减速；夹角大于 90 度，必须停下 (速度为0)
double calculate_junction_speed(TrajectorySegment_t *prev, TrajectorySegment_t *curr) {
    // 计算向量点积 (夹角余弦值 cos_theta)
    /*double cos_theta = (prev->dir_x * curr->dir_x) + 
                       (prev->dir_y * curr->dir_y) + 
                       (prev->dir_z * curr->dir_z);*/
    double cos_theta=0.0;
    for(int i=0;i<AXIS_NUM;i++){
        cos_theta+=(prev->dir_vec[i]*curr->dir_vec[i]);
    }
                       
    // 如果几乎是同向直线 (cos_theta 接近 1)，全速通过！
    if (cos_theta > 0.999) {
        return fmin(prev->v_max, curr->v_max);
    }
    // 如果是锐角或折返 (cos_theta < 0)，为了机械安全，速度降为 0！
    if (cos_theta < 0.0) {
        return 0.0; 
    }
    // 其他情况：按角度比例减速过弯
    double v_min = fmin(prev->v_max, curr->v_max);
    return v_min * cos_theta; 
}

// ====================================================================
// Look-Ahead 前瞻核心引擎 速度规划器 (反向+正向扫描)
// ====================================================================
/*void planner_recalculate() 
{
    int count = (g_cmd_queue.head - g_cmd_queue.tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count == 0) return; 
    if (count < 3 ){
        int curr=g_cmd_queue.tail;
        while(curr!=g_cmd_queue.head){
            g_cmd_queue.buffer[curr].is_ready=1;
            curr=(curr+1)%QUEUE_SIZE;
        }
        return;
    }

    int tail = g_cmd_queue.tail;
    int head = g_cmd_queue.head;

    int safe_tail=(tail+2)%QUEUE_SIZE; // 保持至少2条预留空间，避免正在执行的段被重算导致不稳定
    int curr = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE; // 从最新的一条开始往回算

    // 默认最后一条线一定刹车到 0
    g_cmd_queue.buffer[curr].v_end = 0.0;

    // ------------------------------------------
    // 1. 反向扫描 (确保刹车距离足够)
    // ------------------------------------------
    while (curr != safe_tail) {
        int prev = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev];
        TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];

        // 计算拐角理论允许最高速度
        double v_junc = calculate_junction_speed(s_prev, s_curr);
        
        // 物理反推：以 v_end 结束，最多允许的入弯速度
        double max_v_start = sqrt(pow(s_curr->v_end, 2) + 2.0 * s_curr->dec * s_curr->total_distance);
        
        // 木桶原理取最低限制
        s_curr->v_start = fmin(v_junc, max_v_start);
        s_curr->v_start = fmin(s_curr->v_start, s_curr->v_target);
        s_prev->v_end = s_curr->v_start; // 交接点速度同步
        
        curr = prev;
    }

    // ------------------------------------------
    // 2. 正向扫描 (确保加速合法)
    // ------------------------------------------
    curr = safe_tail;
    g_cmd_queue.buffer[curr].v_start = 0.0; // 起步假定为0 (可优化为获取真实速度)
    
    while (curr != (head - 1 + QUEUE_SIZE) % QUEUE_SIZE) {
        TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];
        int next = (curr + 1) % QUEUE_SIZE;
        TrajectorySegment_t *s_next = &g_cmd_queue.buffer[next];

        // 物理正推：以 v_start 起步，最多能加速到的速度
        double max_v_end = sqrt(pow(s_curr->v_start, 2) + 2.0 * s_curr->acc * s_curr->total_distance);
        
        s_curr->v_end = fmin(s_curr->v_end, max_v_end);
        s_curr->v_end = fmin(s_curr->v_end, s_curr->v_target);
        s_next->v_start = s_curr->v_end;
        
        curr = next;
    }

    // ------------------------------------------
    // 3. 时间重构 (重新计算给底层小脑用的时间分配)
    // ------------------------------------------
    curr = tail;
    while (curr != head) {
        TrajectorySegment_t *seg = &g_cmd_queue.buffer[curr];
        
        double v_s = seg->v_start;
        double v_e = seg->v_end;
        double v_m = seg->v_target;
        double a = seg->acc;
        double d = seg->dec;
        double S = seg->total_distance;

        // 计算达到最高速需要的加速和减速距离
        double d_acc = fabs(pow(v_m, 2) - pow(v_s, 2)) / (2.0 * a);
        double d_dec = fabs(pow(v_m, 2) - pow(v_e, 2)) / (2.0 * d);

        if (d_acc + d_dec > S) {
            // 跑不到最高速，退化为三角形曲线 (初中物理极值公式)
            v_m = sqrt((2.0 * S * a * d + pow(v_s, 2) * d + pow(v_e, 2) * a) / (a + d));
            seg->v_target = v_m; // 更新实际最高速度
            seg->t_acc = (int32_t)ceil(fabs(v_m - v_s) / a);
            seg->t_dec = (int32_t)ceil(fabs(v_m - v_e) / d);
            seg->t_cru = 0;
        } else {
            // 标准梯形曲线
            seg->t_acc = (int32_t)ceil(fabs(v_m - v_s) / a);
            seg->t_dec = (int32_t)ceil(fabs(v_m - v_e) / d);
            double d_cru = S - d_acc - d_dec;
            seg->t_cru = (int32_t)ceil(d_cru / v_m);
        }
        
        seg->t_total = seg->t_acc + seg->t_cru + seg->t_dec;
        if(seg->t_total <= 0) seg->t_total = 1; // 防除零兜底
        
        curr = (curr + 1) % QUEUE_SIZE;
    }
    __sync_synchronize(); // 确保时间计算结果对实时线程可见

    int safe_release_head=(head-2+QUEUE_SIZE)%QUEUE_SIZE; // 最多保留2条在前面，避免正在执行的段被重算导致不稳定
    int mark_curr=tail;
    while(mark_curr!=safe_release_head){
        g_cmd_queue.buffer[mark_curr].is_ready=1; // 标记为准备好，实时线程可以开始执行了
        mark_curr=(mark_curr+1)%QUEUE_SIZE;
    }
}*/

void planner_recalculate() 
{
    int count = (g_cmd_queue.head - g_cmd_queue.tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count == 0) return; 

    int tail = g_cmd_queue.tail;
    int head = g_cmd_queue.head;

    // ==========================================
    // 只有线段足够多(>=3)，才进行前瞻速度平滑计算
    // ==========================================
    if (count >= 3) {
        int safe_tail=(tail+2)%QUEUE_SIZE; 
        int curr = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE; 
        g_cmd_queue.buffer[curr].v_end = 0.0;

        // 1. 反向扫描
        while (curr != safe_tail) {
            int prev = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev];
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];

            double v_junc = calculate_junction_speed(s_prev, s_curr);
            double max_v_start = sqrt(pow(s_curr->v_end, 2) + 2.0 * s_curr->dec * s_curr->total_distance);
            
            s_curr->v_start = fmin(v_junc, max_v_start);
            s_curr->v_start = fmin(s_curr->v_start, s_curr->v_target);
            s_prev->v_end = s_curr->v_start; 
            curr = prev;
        }

        // 2. 正向扫描
        curr = safe_tail;
        g_cmd_queue.buffer[curr].v_start = 0.0; 
        while (curr != (head - 1 + QUEUE_SIZE) % QUEUE_SIZE) {
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];
            int next = (curr + 1) % QUEUE_SIZE;
            TrajectorySegment_t *s_next = &g_cmd_queue.buffer[next];

            double max_v_end = sqrt(pow(s_curr->v_start, 2) + 2.0 * s_curr->acc * s_curr->total_distance);
            s_curr->v_end = fmin(s_curr->v_end, max_v_end);
            s_curr->v_end = fmin(s_curr->v_end, s_curr->v_target);
            s_next->v_start = s_curr->v_end;
            curr = next;
        }
    }

    // ==========================================
    // 3. 时间重构 (极其关键：哪怕只有1条线，也必须算时间！)
    // ==========================================
    int curr = tail;
    while (curr != head) {
        TrajectorySegment_t *seg = &g_cmd_queue.buffer[curr];
        
        double v_s = seg->v_start;
        double v_e = seg->v_end;
        double v_m = seg->v_target;
        double a = seg->acc;
        double d = seg->dec;
        double S = seg->total_distance;

        double d_acc = fabs(pow(v_m, 2) - pow(v_s, 2)) / (2.0 * a);
        double d_dec = fabs(pow(v_m, 2) - pow(v_e, 2)) / (2.0 * d);

        if (d_acc + d_dec > S) {
            v_m = sqrt((2.0 * S * a * d + pow(v_s, 2) * d + pow(v_e, 2) * a) / (a + d));
            seg->v_target = v_m; 
            seg->t_acc = (int32_t)ceil(fabs(v_m - v_s) / a);
            seg->t_dec = (int32_t)ceil(fabs(v_m - v_e) / d);
            seg->t_cru = 0;
        } else {
            seg->t_acc = (int32_t)ceil(fabs(v_m - v_s) / a);
            seg->t_dec = (int32_t)ceil(fabs(v_m - v_e) / d);
            double d_cru = S - d_acc - d_dec;
            seg->t_cru = (int32_t)ceil(d_cru / v_m);
        }
        
        seg->t_total = seg->t_acc + seg->t_cru + seg->t_dec;
        if(seg->t_total <= 0) seg->t_total = 1; 
        
        curr = (curr + 1) % QUEUE_SIZE;
    }
    __sync_synchronize(); 

    // ==========================================
    // 4. 释放标志
    // ==========================================
    if (count < 3) {
        // 如果线段少，没法做完美前瞻，直接放行当前所有指令让底层跑完
        int mark_curr = tail;
        while(mark_curr != head){
            g_cmd_queue.buffer[mark_curr].is_ready = 1;
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    } else {
        // 线段充足，保留最后2条作为将来前瞻的衔接点
        int safe_release_head = (head - 2 + QUEUE_SIZE) % QUEUE_SIZE; 
        int mark_curr = tail;
        while(mark_curr != safe_release_head){
            g_cmd_queue.buffer[mark_curr].is_ready = 1; 
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    }
}