#include "planner.h"

// ====================================================================
// G64 向心加速度过弯模型（替代旧 cos_theta 线性降速）
// @Context: Non-RealTime Background Thread (caller holds planner_mutex)
// ====================================================================
double calculate_junction_speed(TrajectorySegment_t *prev, TrajectorySegment_t *curr)
{
    // ---- M 代码安全隔离墙 ----
    if (prev->cmd_type == CMD_TYPE_MCODE || curr->cmd_type == CMD_TYPE_MCODE) {
        return 0.0;
    }

    // ---- N 维方向向量点积 ----
    double cos_theta = 0.0;
    for (int i = 0; i < AXIS_NUM; i++) {
        cos_theta += prev->dir_vec[i] * curr->dir_vec[i];
    }

    // 钳制到 [-1.0, 1.0]，防止浮点漂移导致 acos NaN
    if (cos_theta > 1.0)  cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    // ---- 极端角度快速路径 ----
    if (cos_theta >= 0.999) {
        return fmin(prev->v_max, curr->v_max);
    }
    if (cos_theta <= -0.999) {
        return 0.0;
    }

    // ---- 向心加速度模型 ----
    double alpha = acos(cos_theta);
    double denom = 1.0 - cos(alpha * 0.5);
    if (denom <= 1e-6) denom = 1e-6;

    double v_allow = sqrt(g_planner_config.max_centripetal_acc
                        * (g_planner_config.corner_tolerance / denom));

    // 防爆墙：NaN / Inf / 负数强制归零
    if (isnan(v_allow) || isinf(v_allow) || v_allow < 0.0) {
        v_allow = 0.0;
    }

    double v_min = fmin(prev->v_max, curr->v_max);
    return fmin(v_allow, v_min);
}

// ====================================================================
// Look-Ahead 前瞻核心引擎（反向+正向扫描 + plan_tail 结界 + 互斥锁）
// @Context: Non-RealTime Background Thread (parser 或 看门狗)
// @Thread-Safety: 由 planner_mutex 保护，禁止 RT 线程调用！
// force_flush: 0=正常前瞻, 1=强制释放所有未释放段（饥饿唤醒用）
// ====================================================================
void planner_recalculate(int force_flush)
{
    pthread_mutex_lock(&planner_mutex);

    int tail = g_cmd_queue.tail;
    int head = g_cmd_queue.head;
    int count = (head - tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count == 0) {
        pthread_mutex_unlock(&planner_mutex);
        return;
    }

    // ================================================================
    // plan_tail 结界：跳过所有 is_ready==1 的已释放段
    // 已释放段正在被 RT 线程消费，神圣不可侵犯！
    // ================================================================
    int plan_tail = tail;
    while (plan_tail != head && atomic_load_explicit(&g_cmd_queue.buffer[plan_tail].is_ready, memory_order_acquire) == 1) {
        plan_tail = (plan_tail + 1) % QUEUE_SIZE;
    }
    int plan_count = (head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (plan_count == 0) {
        pthread_mutex_unlock(&planner_mutex);
        return;
    }

    // ---- 0. 仅初始化未释放段的 v_start / v_end ----
    {
        int c = plan_tail;
        while (c != head) {
            if (atomic_load_explicit(&g_cmd_queue.buffer[c].is_ready, memory_order_acquire) == 0) {
                g_cmd_queue.buffer[c].v_start = 0.0;
                g_cmd_queue.buffer[c].v_end   = 0.0;
            }
            c = (c + 1) % QUEUE_SIZE;
        }
    }

    // ================================================================
    // 前瞻速度平滑计算（plan_count >= 3 才启动）
    // ================================================================
    if (plan_count >= 3) {
        int safe_tail = (plan_tail + 1) % QUEUE_SIZE;
        int curr = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        g_cmd_queue.buffer[curr].v_end = 0.0;

        // ---- 1. 反向扫描（head-1 → safe_tail，确保刹车距离足够）----
        while (curr != safe_tail) {
            int prev = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev];
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];

            double v_junc = calculate_junction_speed(s_prev, s_curr);

            double inner = s_curr->v_end * s_curr->v_end
                         + 2.0 * s_curr->dec * s_curr->total_distance;
            if (inner < 0.0) inner = 0.0;
            double max_v_start = sqrt(inner);

            s_curr->v_start = fmin(v_junc, max_v_start);
            s_curr->v_start = fmin(s_curr->v_start, s_curr->v_target);
            s_prev->v_end = s_curr->v_start;
            curr = prev;
        }
        // 反向扫描结束后：plan_tail.v_end 已通过传播被设置

        // ---- 计算 plan_tail 的 v_start（与已释放前驱的衔接速度）----
        if (plan_tail != tail) {
            // 存在已释放前驱段，用拐角衔接速度
            int prev_idx = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            double v_junc = calculate_junction_speed(
                &g_cmd_queue.buffer[prev_idx], &g_cmd_queue.buffer[plan_tail]);

            double inner = g_cmd_queue.buffer[plan_tail].v_end
                         * g_cmd_queue.buffer[plan_tail].v_end
                         + 2.0 * g_cmd_queue.buffer[plan_tail].dec
                         * g_cmd_queue.buffer[plan_tail].total_distance;
            if (inner < 0.0) inner = 0.0;
            double max_v_start = sqrt(inner);

            g_cmd_queue.buffer[plan_tail].v_start = fmin(v_junc, max_v_start);
            g_cmd_queue.buffer[plan_tail].v_start = fmin(
                g_cmd_queue.buffer[plan_tail].v_start,
                g_cmd_queue.buffer[plan_tail].v_target);
        }
        // else: plan_tail == tail，v_start 保持初始化值 0.0（从静止起步）

        // ---- 2. 正向扫描（safe_tail → head-1，确保加速合法）----
        // 正向扫描起点继承 plan_tail.v_end（衔接速度）
        curr = safe_tail;
        g_cmd_queue.buffer[curr].v_start = g_cmd_queue.buffer[plan_tail].v_end;

        while (curr != (head - 1 + QUEUE_SIZE) % QUEUE_SIZE) {
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];
            int next = (curr + 1) % QUEUE_SIZE;
            TrajectorySegment_t *s_next = &g_cmd_queue.buffer[next];

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

    // ================================================================
    // 衔接速度保障：plan_count < 3 时补齐段间衔接
    // ================================================================

    // plan_count == 2: 两条段之间做 G64 拐角平滑（含动力学约束）
    if (plan_count == 2) {
        int seg0 = plan_tail;
        int seg1 = (plan_tail + 1) % QUEUE_SIZE;
        TrajectorySegment_t *s0 = &g_cmd_queue.buffer[seg0];
        TrajectorySegment_t *s1 = &g_cmd_queue.buffer[seg1];

        double v_junc = calculate_junction_speed(s0, s1);
        v_junc = fmin(v_junc, fmin(s0->v_target, s1->v_target));

        // s0 加速距离约束：从 v_start 出发，最多能加速到多少
        double inner0 = s0->v_start * s0->v_start
                      + 2.0 * fmax(s0->acc, 1e-6) * s0->total_distance;
        if (inner0 < 0.0) inner0 = 0.0;
        double max_v_end_by_acc = sqrt(inner0);
        s0->v_end = fmin(v_junc, max_v_end_by_acc);
        s0->v_end = fmin(s0->v_end, s0->v_target);

        // s1 减速距离约束：从 v_junc 出发，必须能刹停到 v_end
        double inner1 = s1->v_end * s1->v_end
                      + 2.0 * fmax(s1->dec, 1e-6) * s1->total_distance;
        if (inner1 < 0.0) inner1 = 0.0;
        double max_v_start_by_dec = sqrt(inner1);
        s1->v_start = fmin(s0->v_end, max_v_start_by_dec);
        s1->v_start = fmin(s1->v_start, s1->v_target);

        s0->v_end = s1->v_start; // 链式约束：保证衔接一致

        // 如果 plan_tail 有已释放前驱，还需计算 plan_tail.v_start
        if (plan_tail != tail) {
            int prev_idx = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            double v_junc_front = calculate_junction_speed(
                &g_cmd_queue.buffer[prev_idx], s0);

            double inner_f = s0->v_end * s0->v_end
                           + 2.0 * fmax(s0->dec, 1e-6) * s0->total_distance;
            if (inner_f < 0.0) inner_f = 0.0;
            double max_v_start = sqrt(inner_f);

            s0->v_start = fmin(v_junc_front, max_v_start);
            s0->v_start = fmin(s0->v_start, s0->v_target);
        }
    }

    // plan_count == 1 且有前驱：补齐 plan_tail 与前驱的衔接
    if (plan_count == 1 && plan_tail != tail) {
        int prev_idx = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        TrajectorySegment_t *s_plan = &g_cmd_queue.buffer[plan_tail];

        double v_junc = calculate_junction_speed(
            &g_cmd_queue.buffer[prev_idx], s_plan);

        double inner = s_plan->v_end * s_plan->v_end
                     + 2.0 * s_plan->dec * s_plan->total_distance;
        if (inner < 0.0) inner = 0.0;
        double max_v_start = sqrt(inner);

        s_plan->v_start = fmin(v_junc, max_v_start);
        s_plan->v_start = fmin(s_plan->v_start, s_plan->v_target);
    }

    // ================================================================
    // 3. 时间重构（仅对未释放段计算，double 中间量 + 30s 熔断）
    // ================================================================
    {
        int curr = plan_tail;
        while (curr != head) {
            TrajectorySegment_t *seg = &g_cmd_queue.buffer[curr];

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
            double a   = fmax(seg->acc, 1e-6);
            double d   = fmax(seg->dec, 1e-6);
            double S   = seg->total_distance;

            double d_acc = fabs(v_m * v_m - v_s * v_s) / (2.0 * a);
            double d_dec = fabs(v_m * v_m - v_e * v_e) / (2.0 * d);

            double t_acc_d, t_dec_d, t_cru_d;

            if (d_acc + d_dec > S) {
                double inner = (2.0 * S * a * d + v_s * v_s * d + v_e * v_e * a) / (a + d);
                if (inner < 0.0) inner = 0.0;
                v_m = sqrt(inner);
                seg->v_target = v_m;
                t_acc_d = ceil(fabs(v_m - v_s) / a);
                t_dec_d = ceil(fabs(v_m - v_e) / d);
                t_cru_d = 0.0;
            } else {
                t_acc_d = ceil(fabs(v_m - v_s) / a);
                t_dec_d = ceil(fabs(v_m - v_e) / d);
                double d_cru = S - d_acc - d_dec;
                t_cru_d = (v_m > 1e-6) ? ceil(d_cru / v_m) : 0.0;
            }

            double t_total_d = t_acc_d + t_dec_d + t_cru_d;

            // 拦截 NaN / Inf 数学异常，或 t_total 溢出 int32_t
            // 溢出说明动力学参数极其荒谬（耗时超 24 天），直接废弃
            if (isnan(t_total_d) || isinf(t_total_d) || t_total_d > (double)INT32_MAX) {
                seg->t_acc   = 0;
                seg->t_dec   = 0;
                seg->t_cru   = 0;
                seg->t_total = 1;
                seg->total_distance = 0.0;
                seg->v_target = 0.0;
                seg->acc = 0.0;
                seg->dec = 0.0;
                curr = (curr + 1) % QUEUE_SIZE;
                continue;
            }

            seg->t_acc   = (int32_t)t_acc_d;
            seg->t_dec   = (int32_t)t_dec_d;
            seg->t_cru   = (int32_t)t_cru_d;
            seg->t_total = (int32_t)t_total_d;
            if (seg->t_total <= 0) seg->t_total = 1;

            curr = (curr + 1) % QUEUE_SIZE;
        }
    }

    // ================================================================
    // 4. 释放标志（release 语义确保上方所有写入对 RT 线程可见）
    // ================================================================
    if (force_flush || plan_count < 3) {
        int mark_curr = plan_tail;
        while (mark_curr != head) {
            atomic_store_explicit(&g_cmd_queue.buffer[mark_curr].is_ready, 1, memory_order_release);
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    } else {
        int safe_release_head = (head - 2 + QUEUE_SIZE) % QUEUE_SIZE;
        // 如果最后一段是 M 代码，它要求停顿执行，前驱段无需保留做速度平滑
        if (g_cmd_queue.buffer[(head - 1 + QUEUE_SIZE) % QUEUE_SIZE].cmd_type == CMD_TYPE_MCODE) {
            safe_release_head = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        }
        int mark_curr = plan_tail;
        while (mark_curr != safe_release_head) {
            atomic_store_explicit(&g_cmd_queue.buffer[mark_curr].is_ready, 1, memory_order_release);
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    }

    pthread_mutex_unlock(&planner_mutex);
}
