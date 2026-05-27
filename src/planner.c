#include "planner.h"

// ====================================================================
// G64 向心加速度过弯模型（替代旧 cos_theta 线性降速）
// ====================================================================
double calculate_junction_speed(TrajectorySegment_t *prev, TrajectorySegment_t *curr)
{
    // ---- 第一步：M 代码安全隔离墙 ----
    if (prev->cmd_type == CMD_TYPE_MCODE || curr->cmd_type == CMD_TYPE_MCODE) {
        return 0.0;
    }

    // ---- 第一步：计算 N 维方向向量点积 ----
    double cos_theta = 0.0;
    for (int i = 0; i < AXIS_NUM; i++) {
        cos_theta += prev->dir_vec[i] * curr->dir_vec[i];
    }

    // 钳制到 [-1.0, 1.0]，防止浮点漂移导致 acos NaN
    if (cos_theta > 1.0)  cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    // ---- 第二步：极端角度快速路径 ----
    if (cos_theta >= 0.999) {
        // 接近直线，全速通过
        return fmin(prev->v_max, curr->v_max);
    }
    if (cos_theta <= -0.999) {
        // 完全折返，必须停稳
        return 0.0;
    }

    // ---- 第三步：向心加速度模型 ----
    const double TOLERANCE          = 0.05;   // 拐角允许的最大物理偏离误差 (mm)
    const double MAX_CENTRIPETAL_ACC = 500.0;  // 最大向心加速度 (mm/s^2)

    double alpha = acos(cos_theta);            // 两线段夹角（弧度）
    double denom = 1.0 - cos(alpha * 0.5);     // 内切圆半径分母
    if (denom <= 1e-6) denom = 1e-6;           // 防除零

    double v_allow = sqrt(MAX_CENTRIPETAL_ACC * (TOLERANCE / denom));

    // ---- 第四步：取三者最小值 ----
    double v_min = fmin(prev->v_max, curr->v_max);
    return fmin(v_allow, v_min);
}

// ====================================================================
// Look-Ahead 前瞻核心引擎（反向+正向扫描 + 安全加固）
// ====================================================================
void planner_recalculate()
{
    int count = (g_cmd_queue.head - g_cmd_queue.tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count == 0) return;

    int tail = g_cmd_queue.tail;
    int head = g_cmd_queue.head;

    // ---- 0. 初始化未计算段的 v_start / v_end，消灭野数据 ----
    {
        int c = tail;
        while (c != head) {
            if (g_cmd_queue.buffer[c].is_ready == 0) {
                g_cmd_queue.buffer[c].v_start = 0.0;
                g_cmd_queue.buffer[c].v_end   = 0.0;
            }
            c = (c + 1) % QUEUE_SIZE;
        }
    }

    // ==========================================
    // 只有线段足够多(>=3)，才进行前瞻速度平滑计算
    // ==========================================
    if (count >= 3) {
        int safe_tail = (tail + 1) % QUEUE_SIZE;
        int curr = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        g_cmd_queue.buffer[curr].v_end = 0.0;

        // ---- 1. 反向扫描（确保刹车距离足够）----
        while (curr != safe_tail) {
            int prev = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev];
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];

            double v_junc = calculate_junction_speed(s_prev, s_curr);

            // 物理 反推 ：以 v_end 结束，最多允许的入弯速度（pow→乘法，sqrt 防负）
            double inner = s_curr->v_end * s_curr->v_end
                         + 2.0 * s_curr->dec * s_curr->total_distance;
            if (inner < 0.0) inner = 0.0;
            double max_v_start = sqrt(inner);

            s_curr->v_start = fmin(v_junc, max_v_start);
            s_curr->v_start = fmin(s_curr->v_start, s_curr->v_target);
            s_prev->v_end = s_curr->v_start;
            curr = prev;
        }

        // ---- 2. 正向扫描（确保加速合法）----
        curr = safe_tail;
        g_cmd_queue.buffer[curr].v_start = 0.0;
        while (curr != (head - 1 + QUEUE_SIZE) % QUEUE_SIZE) {
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];
            int next = (curr + 1) % QUEUE_SIZE;
            TrajectorySegment_t *s_next = &g_cmd_queue.buffer[next];

            // 物理 正推 ：以 v_start 起步，最多能加速到的速度
            double inner = s_curr->v_start * s_curr->v_start
                         + 2.0 * s_curr->acc * s_curr->total_distance;
            if (inner < 0.0) inner = 0.0;
            double max_v_end = sqrt(inner);

            s_curr->v_end = fmin(s_curr->v_end, max_v_end);
            s_curr->v_end = fmin(s_curr->v_end, s_curr->v_target);
            s_next->v_start = s_curr->v_end;
            curr = next;
        }
    }

    // ==========================================
    // 3. 时间重构（极其关键：哪怕只有1条线，也必须算时间）
    // ==========================================
    int curr = tail;
    while (curr != head) {
        TrajectorySegment_t *seg = &g_cmd_queue.buffer[curr];

        // M 代码或零距离段：跳过数学计算，直接置零
        if (seg->cmd_type == CMD_TYPE_MCODE || seg->total_distance <= 0.0001) {
            seg->t_acc   = 0;
            seg->t_dec   = 0;
            seg->t_cru   = 0;
            seg->t_total = 1;
            curr = (curr + 1) % QUEUE_SIZE;
            continue;
        }

        double v_s = seg->v_start;
        double v_e = seg->v_end;
        double v_m = seg->v_target;
        double a   = fmax(seg->acc, 1e-6);   // 除零保护
        double d   = fmax(seg->dec, 1e-6);   // 除零保护
        double S   = seg->total_distance;

        // 计算达到最高速需要的加速和减速距离（pow→乘法）
        double d_acc = fabs(v_m * v_m - v_s * v_s) / (2.0 * a);
        double d_dec = fabs(v_m * v_m - v_e * v_e) / (2.0 * d);

        if (d_acc + d_dec > S) {
            // 跑不到最高速，退化为三角形曲线
            double inner = (2.0 * S * a * d + v_s * v_s * d + v_e * v_e * a) / (a + d);
            if (inner < 0.0) inner = 0.0;
            v_m = sqrt(inner);
            seg->v_target = v_m;
            seg->t_acc = (int32_t)ceil(fabs(v_m - v_s) / a);
            seg->t_dec = (int32_t)ceil(fabs(v_m - v_e) / d);
            seg->t_cru = 0;
        } else {
            // 标准梯形曲线
            seg->t_acc = (int32_t)ceil(fabs(v_m - v_s) / a);
            seg->t_dec = (int32_t)ceil(fabs(v_m - v_e) / d);
            double d_cru = S - d_acc - d_dec;
            if (v_m > 1e-6) {
                seg->t_cru = (int32_t)ceil(d_cru / v_m);
            } else {
                seg->t_cru = 0;
            }
        }

        seg->t_total = seg->t_acc + seg->t_cru + seg->t_dec;
        if (seg->t_total <= 0) seg->t_total = 1;

        curr = (curr + 1) % QUEUE_SIZE;
    }
    __sync_synchronize();

    // ==========================================
    // 4. 释放标志
    // ==========================================
    if (count < 3) {
        int mark_curr = tail;
        while (mark_curr != head) {
            g_cmd_queue.buffer[mark_curr].is_ready = 1;
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    } else {
        int safe_release_head = (head - 2 + QUEUE_SIZE) % QUEUE_SIZE;
        int mark_curr = tail;
        while (mark_curr != safe_release_head) {
            g_cmd_queue.buffer[mark_curr].is_ready = 1;
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    }
}
