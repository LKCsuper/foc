/*
 * @Description: FOC 中断控制
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-01 10:28:50
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-24 23:29:19
 *
 * 这一路只做开环启动/调试用的 FOC：
 *   1. 中断里采样三相电流、相电压、母线电压
 *   2. 软件积分产生开环电角度
 *   3. 根据 ol_mode 选择 V/F 电压开环，或者角度开环 + 电流闭环
 *   4. 统一走反 Park + SVPWM 输出到 TIM1
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"

/* 电流零点正常应在 ADC 中点附近。flash 没校准或读出异常时，用默认值兜底。 */
#define MOTOR_CURRENT_OFFSET_MIN      (1000.0f)
#define MOTOR_CURRENT_OFFSET_MAX      (3000.0f)
#define MOTOR_CURRENT_OFFSET_DEFAULT  (2045.0f)

/* DQ 轴电压中间量，用于电流环 PI 输出与限幅之间的传递。 */
typedef struct {
    float vd; /* D 轴电压 (励磁分量) */
    float vq; /* Q 轴电压 (转矩分量) */
} motor_dq_voltage_t;

/* 起步阶段统一状态，由 Motor_OpenLoop_StartupVf / StartupIf 维护，供下游函数消费。 */
static struct {
    bool  was_stopped;       /* 上次处于停止状态，启动第一拍触发相位补偿 */
    float angle_rpm;         /* 角度积分用转速 */
    float iq_scale;          /* Iq 缩放系数（1.0 = 全量，<1.0 = 低速收小） */
    float phase_boost;       /* 本周期相位补偿量 rad（仅电流模式，一次性） */
} s_ol_startup = {
    .was_stopped = true,
    .angle_rpm   = 0.0f,
    .iq_scale    = 1.0f,
    .phase_boost = 0.0f,
};

/*
 * 计算开环 FOC 的控制周期 dt。
 * 优先从 TIM1 当前 ARR/PSC 反推 ADC-DMA 控制触发频率，再按
 * FOC_CONTROL_LOOP_FREQ_DIVIDER 放大到实际控制周期。
 */
static float Motor_OpenLoop_GetDt(MOTOR_PARAM_S *m)
{
    uint32_t arr = TIM1->ARR;
    uint32_t psc = TIM1->PSC + 1U;
    float foc_freq;

    /*
     * VESC style: derive dt from the real timer rate that drives ADC/DMA.
     * In this project TIM1->ARR is configured as SYSTEM_CORE_CLOCK / f_zv,
     * and TIM1 update events are used as the control-loop trigger source.
     */
    if (arr > 0U && psc > 0U) {
        foc_freq = (float)SYSTEM_CORE_CLOCK / ((float)arr * (float)psc);
        m->ol_foc_freq = foc_freq;
    } else {
        foc_freq = m->ol_foc_freq;
        if (foc_freq <= 1.0f) {
            foc_freq = 1.0f;
            m->ol_foc_freq = foc_freq;
        }
    }

    return (float)FOC_CONTROL_LOOP_FREQ_DIVIDER / foc_freq;
}

/*
 * 获取电压限幅值。
 * 取用户配置 ol_vq_limit 与母线电压 75% 的较小值，
 * 防止过调制导致电流环 PI 积分饱和。
 */
static float Motor_OpenLoop_GetVoltageLimit(MOTOR_PARAM_S *m, float v_bus)
{
    float limit = m->ol_vq_limit;

    if (limit < 0.0f) {
        limit = 0.0f;
    }

    /*
     * SVPWM 这里保守限制到母线 75%。
     * 电流环的 PI 输出最终也会被这个限幅约束，防止过调制后 PI 继续积起来。
     */
    if (limit > v_bus * 0.75f) {
        limit = v_bus * 0.75f;
    }

    return limit;
}

/*
 * 获取有效的电流零点偏移值。
 * 若 flash 中存储的偏移量超出合理范围 (1.0V~3.0V 对应 ADC 中点附近)，
 * 则返回默认值 2045 兜底，避免用异常偏移量算出离谱的电流值。
 */
static float Motor_CurrentOffset_GetValid(float offset)
{
    if (offset < MOTOR_CURRENT_OFFSET_MIN || offset > MOTOR_CURRENT_OFFSET_MAX) {
        return MOTOR_CURRENT_OFFSET_DEFAULT;
    }

    return offset;
}

/*
 * 输出安全状态 (50% 占空比，三相中点电平)。
 * 此时逆变器上下桥臂各占一半，电机绕组等效短接，不产生转矩。
 * 用于停机、欠压、未就绪等保护场景。
 */
static void Motor_OpenLoop_OutputSafe(MOTOR_PARAM_S *m)
{
    m->duty1 = TIM1->ARR / 2;
    m->duty2 = TIM1->ARR / 2;
    m->duty3 = TIM1->ARR / 2;
    TIMER_UPDATE_DUTY_M1(m->duty1, m->duty2, m->duty3);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

/*
 * 复位开环运行时状态：
 *   - 当前转速清零
 *   - D/Q 轴 PI 积分项清零
 *   - 停机标志置位 (下次启动可重新做相位补偿)
 */
static void Motor_OpenLoop_ResetRuntime(MOTOR_PARAM_S *m)
{
    m->ol_current_rpm = 0.0f;
    m->ol_vd_int = 0.0f;
    m->ol_vq_int = 0.0f;
    s_ol_startup.was_stopped = true;
}

/*
 * 停止开环运行：
 *   目标转速归零 → 复位运行时状态 → 输出安全电平。
 */
static void Motor_OpenLoop_Stop(MOTOR_PARAM_S *m)
{
    m->ol_target_rpm = 0.0f;
    Motor_OpenLoop_ResetRuntime(m);
    Motor_OpenLoop_OutputSafe(m);
}

/*
 * 判断开环是否就绪：
 *   1) ol_enable 已使能
 *   2) 目标转速 != 0（正反向均可）
 *   3) 母线电压 >= 1V (避免除零和异常输出)
 */
static bool Motor_OpenLoop_IsReady(MOTOR_PARAM_S *m, float v_bus)
{
    return m->ol_enable && m->ol_target_rpm != 0.0f && v_bus >= 1.0f;
}

/*
 * 对用户配置参数做合法性钳位：
 *   - 极对数 >= 1
 *   - 最低转速/爬坡速率 >= 0
 *   - 最高转速 >= 1
 *   - 目标转速不超过 |最高转速|（正向/反向）
 * 防止异常参数导致除零或失控。
 */
static void Motor_OpenLoop_NormalizeConfig(MOTOR_PARAM_S *m)
{
    if (m->ol_pole_pairs < 1.0f) {
        m->ol_pole_pairs = 1.0f;
    }

    if (m->ol_min_rpm < 0.0f) {
        m->ol_min_rpm = 0.0f;
    }

    if (m->ol_ramp_rpm_s < 0.0f) {
        m->ol_ramp_rpm_s = 0.0f;
    }

    if (m->ol_max_rpm <= 0.0f) {
        m->ol_max_rpm = 1.0f;
    }

    if (m->ol_target_rpm > m->ol_max_rpm) {
        m->ol_target_rpm = m->ol_max_rpm;
    }

    if (m->ol_target_rpm < -m->ol_max_rpm) {
        m->ol_target_rpm = -m->ol_max_rpm;
    }
}

/*
 * 角度积分转速整理。
 * 每个 FOC 周期在 Startup 之后调用一次，统一整理本拍派生状态。
 *
 * 处理内容：
 *   1. 最低转速跳转 — 目标转速超过 ol_min_rpm 但当前 |rpm| 不足时，
 *      angle_rpm 直接跳到 SIGN(target)*ol_min_rpm，保证角度积分有最低频率。
 *   2. 否则 angle_rpm 直接取当前转速 ol_current_rpm。
 *
 * 注意：相位补偿和 Iq 缩放已移到 Motor_OpenLoop_StartupIf 中处理，
 * 本函数只负责角度积分转速，由 UpdateTheta 消费。
 */
static void Motor_OpenLoop_UpdateAngleRpm(MOTOR_PARAM_S *m)
{
    float target = m->ol_target_rpm;
    float rpm = m->ol_current_rpm;

    /*
     * 1. 最低转速跳转 — 目标超过 min_rpm 但当前 |rpm| 不足时，
     *    angle_rpm 跳到 SIGN(target)*min_rpm，保证角度积分有最低频率。
     * 2. 否则 angle_rpm = 当前转速 (目标较低时允许低于 min_rpm)。
     */
    if (fabsf(target) >= m->ol_min_rpm &&
        fabsf(rpm) < m->ol_min_rpm) {
        s_ol_startup.angle_rpm = SIGN(target) * m->ol_min_rpm;
    } else {
        s_ol_startup.angle_rpm = rpm;
    }

    /*
     * 3. 启动相位补偿 — 由 Motor_OpenLoop_StartupIf 在电流模式下处理，
     *    通过 s_ol_startup.phase_boost 写入，Motor_CurrentLoop_GetTheta 消费。
     * 4. 低速 Iq 缩放   — 由 Motor_OpenLoop_StartupIf 在电流模式下处理，
     *    通过 s_ol_startup.iq_scale 写入，Motor_CurrentTarget_Get 消费。
     */
}

/*
 * V/F 模式起步处理：
 *   目标转速超过 ol_min_rpm 时，直接将当前转速跳到 ol_min_rpm，
 *   避免低速段电压不足无法起转。然后委托 UpdateAngleRpm 整理角度积分转速。
 *   V/F 模式不需要相位补偿和 Iq 缩放，直接置为默认值。
 */
static void Motor_OpenLoop_StartupVf(MOTOR_PARAM_S *m)
{
    float target = m->ol_target_rpm;
    float rpm = m->ol_current_rpm;

    s_ol_startup.phase_boost = 0.0f;
    s_ol_startup.iq_scale = 1.0f;

    if (fabsf(rpm) < m->ol_min_rpm &&
        fabsf(target) >= m->ol_min_rpm) {
        m->ol_current_rpm = SIGN(target) * m->ol_min_rpm;
    }

    Motor_OpenLoop_UpdateAngleRpm(m);
}

/*
 * 电流闭环 (I/F) 模式起步处理：
 *   1. 停机后首次启动时，按目标方向补偿一次相位 (phase_boost)，
 *      帮助电流矢量快速对准转子位置，避免起步反转发抖。
 *   2. 低速时按 |rpm|/ol_min_rpm 比例缩放 Iq 目标值 (不低于 10%)，
 *      防止低速反电动势不足时过流。
 *   3. 委托 UpdateAngleRpm 整理角度积分转速 — 电流模式下不跳 ol_current_rpm，
 *      但角度积分转速不低于 ol_min_rpm，保证电流矢量以最低频率旋转。
 */
static void Motor_OpenLoop_StartupIf(MOTOR_PARAM_S *m)
{
    float target = m->ol_target_rpm;
    float rpm = m->ol_current_rpm;

    s_ol_startup.phase_boost = 0.0f;
    if (s_ol_startup.was_stopped && target != 0.0f) {
        s_ol_startup.phase_boost = SIGN(target) * m->ol_start_phase_boost;
        s_ol_startup.was_stopped = false;
    }

    s_ol_startup.iq_scale = 1.0f;
    if (fabsf(rpm) < m->ol_min_rpm &&
        m->ol_min_rpm > 0.0f) {
        float scale = fabsf(rpm) / m->ol_min_rpm;
        if (scale < 0.1f) {
            scale = 0.1f;
        }
        s_ol_startup.iq_scale = scale;
    }

    Motor_OpenLoop_UpdateAngleRpm(m);
}

/*
 * 转速斜坡更新：
 *   按 ol_ramp_rpm_s 速率逼近目标转速，支持正向/反向双向斜坡。
 *   ramp 为 0 时直接跳到目标转速 (无斜坡)。
 *   到达目标后保持。
 */
static void Motor_OpenLoop_UpdateSpeed(MOTOR_PARAM_S *m, float dt)
{
    float target = m->ol_target_rpm;
    float step = m->ol_ramp_rpm_s * dt;

    /*
     * ramp 为 0 时直接跳到目标转速。
     * 否则按 rpm/s 斜坡逼近目标。
     */
    if (step <= 0.0f) {
        m->ol_current_rpm = target;
        return;
    }

    if (m->ol_current_rpm < target) {
        m->ol_current_rpm += step;
        if (m->ol_current_rpm > target) {
            m->ol_current_rpm = target;
        }
    } else if (m->ol_current_rpm > target) {
        m->ol_current_rpm -= step;
        if (m->ol_current_rpm < target) {
            m->ol_current_rpm = target;
        }
    }
}

/*
 * 电角度积分更新：
 *   用 Startup 提供的 angle_rpm 换算为电角速度，累加到 ol_theta。
 *   乘以极对数是因为电角度 = 机械角度 × 极对数。
 */
static void Motor_OpenLoop_UpdateTheta(MOTOR_PARAM_S *m, float dt)
{
    /* 角度积分转速由 Startup 提供。 */
    m->ol_theta += RPM2RADPS_f(s_ol_startup.angle_rpm) * m->ol_pole_pairs * dt;
    utils_norm_angle_rad(&m->ol_theta);
}

/*
 * FOC 电压输出统一入口：
 *   反 Park 变换 (vd/vq → v_alpha/v_beta) → SVPWM 计算占空比 → 更新 TIM1 比较寄存器。
 * 无论 V/F 模式还是电流闭环模式，最终都走这条路输出 PWM。
 */
static void Motor_Foc_OutputVoltage(MOTOR_PARAM_S *m, float vd, float vq,
                                    float theta, float v_bus)
{
    float v_alpha;
    float v_beta;
    ULONG sector;

    inverse_park(vd, vq, theta, &v_alpha, &v_beta);
    svm_hqd(v_alpha, v_beta, v_bus, TIM1->ARR,
            &m->duty1, &m->duty2, &m->duty3, &sector);
    TIMER_UPDATE_DUTY_M1(m->duty1, m->duty2, m->duty3);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

/*
 * V/F 标量开环模式：
 *   根据当前电频率计算 q 轴电压: vq = V_boost + Kv * f_elec
 *   d 轴给 0，仅靠 q 轴电压拖动电机旋转。
 *   适合无电流采样或只需简单转起来的调试场景。
 */
static void Motor_OpenLoop_RunVf(MOTOR_PARAM_S *m, float v_bus)
{
    float vq_limit = Motor_OpenLoop_GetVoltageLimit(m, v_bus);
    float f_elec = fabsf(m->ol_current_rpm) / 60.0f * m->ol_pole_pairs;
    float vq = 0.0f;

    if (m->ol_v_boost < 0.0f) {
        m->ol_v_boost = 0.0f;
    }

    if (m->ol_kv_f < 0.0f) {
        m->ol_kv_f = 0.0f;
    }

    /* V/F：速度越高，反电动势越大，需要给更高的 q 轴电压。 */
    vq = m->ol_v_boost + m->ol_kv_f * f_elec;
    if (vq > vq_limit) {
        vq = vq_limit;
    }

    Motor_Foc_OutputVoltage(m, 0.0f, vq, m->ol_theta, v_bus);
}

/*
 * 获取电流环使用的电角度。
 * 在开环积分角度的基础上叠加一次性相位补偿 (仅启动第一拍非零)，
 * 帮助 I/F 起步时电流矢量尽快对准转子，避免失步。
 * 补偿值由 ol_start_phase_boost 配置，典型值 PI/2 (90°)。
 */
static float Motor_CurrentLoop_GetTheta(MOTOR_PARAM_S *m)
{
    float theta = m->ol_theta + s_ol_startup.phase_boost;

    utils_norm_angle_rad(&theta);
    return theta;
}

/*
 * 电流反馈链路：三相静止坐标系 → 两相旋转坐标系。
 *   Clarke 变换: ia/ib/ic → Ialpha/Ibeta
 *   Park 变换:  Ialpha/Ibeta + theta → Id/Iq
 */
static void Motor_CurrentFeedback_UpdateDq(MOTOR_PARAM_S *m, float theta)
{
    clarke_transform(m->ia, m->ib, m->ic, &m->Ialpha, &m->Ibeta);
    park_transform(m->Ialpha, m->Ibeta, theta, &m->Id, &m->Iq);
}

/*
 * 获取电流环目标值：
 *   Id 目标直接取用户配置 ol_id_set (通常为 0，仅控制励磁)。
 *   Iq 目标取 ol_iq_set 并乘以 Startup 提供的低速缩放系数，
 *   起步阶段逐步放开 Iq，避免电流冲击。
 */
static void Motor_CurrentTarget_Get(MOTOR_PARAM_S *m, float *id_target,
                                    float *iq_target)
{
    *id_target = m->ol_id_set;
    *iq_target = m->ol_iq_set;

    /* 低速 Iq 缩放系数由 Startup 统一计算。 */
    *iq_target *= s_ol_startup.iq_scale;
}

/*
 * 电压矢量限幅 + 积分抗饱和 (Anti-Windup)：
 *   对 vd/vq 合成矢量做幅值限制，超限时按比例整体缩放，
 *   并回退 PI 积分项，防止输出饱和后积分继续累积导致退饱和超调。
 *   id_err/iq_err 用于反算 PI 的 P 项贡献，精确回退到限幅边界。
 */
static void Motor_CurrentLoop_LimitVoltage(MOTOR_PARAM_S *m,
                                           motor_dq_voltage_t *v,
                                           float id_err,
                                           float iq_err,
                                           float v_limit)
{
    /*
     * 单独限制 vd/vq 不够，合成矢量可能超限。
     * 这里按矢量幅值整体缩放，并回写积分项，避免 windup。
     */
    if (utils_saturate_vector_2d(&v->vd, &v->vq, v_limit)) {
        m->ol_vd_int = v->vd - id_err * m->ol_current_kp;
        m->ol_vq_int = v->vq - iq_err * m->ol_current_kp;
    }
}

/*
 * D/Q 轴电流 PI 闭环：
 *   分别对 Id/Iq 跑 PI 控制器 → 得到 vd/vq → 矢量限幅 (防 windup)。
 *   返回限幅后的 (vd, vq) 电压指令。
 */
static motor_dq_voltage_t Motor_CurrentLoop_RunPi(MOTOR_PARAM_S *m,
                                                  float id_target,
                                                  float iq_target,
                                                  float dt,
                                                  float v_limit)
{
    motor_dq_voltage_t v;

    v.vd = Motor_Current_PI_Run(id_target, m->Id, &m->ol_vd_int,
                                m->ol_current_kp, m->ol_current_ki, dt, v_limit);
    v.vq = Motor_Current_PI_Run(iq_target, m->Iq, &m->ol_vq_int,
                                m->ol_current_kp, m->ol_current_ki, dt, v_limit);

    Motor_CurrentLoop_LimitVoltage(m, &v,
                                   id_target - m->Id,
                                   iq_target - m->Iq,
                                   v_limit);
    return v;
}

/*
 * I/F 电流闭环模式主流程：
 *   获取补偿后的电角度 → Clarke/Park 变换得到 Id/Iq 反馈 →
 *   获取目标电流 (含 Iq 缩放) → PI 闭环计算 vd/vq →
 *   矢量限幅 → 反 Park + SVPWM 输出。
 *   此模式下电流闭环跟踪目标，角度仍为开环积分产生。
 */
static void Motor_OpenLoop_RunIf(MOTOR_PARAM_S *m, float v_bus, float dt)
{
    float id_target;
    float iq_target;
    float theta = Motor_CurrentLoop_GetTheta(m);
    float v_limit = Motor_OpenLoop_GetVoltageLimit(m, v_bus);
    motor_dq_voltage_t voltage;

    /*
     * 电流闭环主流程：
     *   三相电流 -> Clarke/Park -> Id/Iq
     *   Id/Iq 误差 -> PI -> vd/vq
     *   vd/vq 限幅 -> 反 Park/SVPWM
     */
    Motor_CurrentFeedback_UpdateDq(m, theta);
    Motor_CurrentTarget_Get(m, &id_target, &iq_target);
    voltage = Motor_CurrentLoop_RunPi(m, id_target, iq_target, dt, v_limit);
    Motor_Foc_OutputVoltage(m, voltage.vd, voltage.vq, theta, v_bus);
}

/*
 * 开环运行主状态机 (每个 FOC 控制周期调用一次)：
 *   1. 欠压 → 复位 + 安全输出
 *   2. 未就绪 → 停机
 *   3. 就绪 → 计算 dt → 更新运动状态 → 根据 ol_mode 走 V/F 或电流闭环
 */
static void Motor_OpenLoop_Run(MOTOR_PARAM_S *m, float v_bus)
{
    float dt;

    /* 欠压时直接复位运行状态，输出三相中点电平。 */
    if (v_bus < 1.0f) {
        Motor_OpenLoop_ResetRuntime(m);
        Motor_OpenLoop_OutputSafe(m);
        return;
    }

    /* 未使能或没有目标转速时保持停机。 */
    if (!Motor_OpenLoop_IsReady(m, v_bus)) {
        Motor_OpenLoop_Stop(m);
        return;
    }

    dt = Motor_OpenLoop_GetDt(m);

    /* 先得到本拍转速，再基于本拍转速计算角度、Iq缩放和相位补偿。 */
    Motor_OpenLoop_NormalizeConfig(m);
    Motor_OpenLoop_UpdateSpeed(m, dt);
    if (m->ol_mode == MOTOR_OL_MODE_IF) {
        Motor_OpenLoop_StartupIf(m);
        Motor_OpenLoop_UpdateTheta(m, dt);
        Motor_OpenLoop_RunIf(m, v_bus, dt);
    } else {
        Motor_OpenLoop_StartupVf(m);
        Motor_OpenLoop_UpdateTheta(m, dt);
        Motor_OpenLoop_RunVf(m, v_bus);
    }
}

/*
 * 分频控制：DMA 中断每次都会进来，但 FOC 算法不需要每拍都跑。
 * 通过静态计数器 skip 做 FOC_CONTROL_LOOP_FREQ_DIVIDER 分频，
 * 只有计数满时才返回 true，触发一次 FOC 运算。
 */
static bool Motor_DmaInt_ShouldRun(void)
{
    static int skip = 0;

    if (++skip >= FOC_CONTROL_LOOP_FREQ_DIVIDER) {
        skip = 0;
        return true;
    }

    return false;
}

/*
 * 中断内参数采样与换算 (每个 FOC 控制周期执行一次)：
 *   1. 读取三相电流 ADC → 减去零点偏移 → 乘以电流系数 → ia/ib/ic
 *   2. 读取三相电压 ADC → 减去未驱动偏移 → 乘以分压比和系数 → va/vb/vc
 *   3. 读取母线电压，经一阶低通滤波 → v_bus
 *
 * 注意：vb/vc 的 ADC 通道编号与相序不是简单的一一对应，见硬件映射。
 */
VOID Motor_It_ParamCal(VOID)
{
    MOTOR_PARAM_S *m = &motor;
    float current_offset0;
    float current_offset1;
    float current_offset2;

    m->currentAdc[0] = GET_CURRENT1();
    m->currentAdc[1] = GET_CURRENT2();
    m->currentAdc[2] = GET_CURRENT3();

    m->voltAdc[0] = ADC_VOLTS(ADC_IND_SENS1);
    m->voltAdc[1] = ADC_VOLTS(ADC_IND_SENS2);
    m->voltAdc[2] = ADC_VOLTS(ADC_IND_SENS3);

    current_offset0 = Motor_CurrentOffset_GetValid(m->currentOffset[0]);
    current_offset1 = Motor_CurrentOffset_GetValid(m->currentOffset[1]);
    current_offset2 = Motor_CurrentOffset_GetValid(m->currentOffset[2]);

    m->ia = (m->currentAdc[0] - current_offset0) * FAC_CURRENT;   
    m->ib = (m->currentAdc[1] - current_offset1) * FAC_CURRENT;
    m->ic = (m->currentAdc[2] - current_offset2) * FAC_CURRENT;

    /*
     * 注意 ADC 通道与相序的映射不是线性的：
     *   va ← ADC_IND_SENS1 (offset[0])
     *   vb ← ADC_IND_SENS3 (offset[2]) — 交换
     *   vc ← ADC_IND_SENS2 (offset[1]) — 交换
     */
    m->va = (ADC_VOLTS(ADC_IND_SENS1) - m->voltundrivenOffset[0]) *
            ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    m->vb = (ADC_VOLTS(ADC_IND_SENS3) - m->voltundrivenOffset[2]) *
            ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
    m->vc = (ADC_VOLTS(ADC_IND_SENS2) - m->voltundrivenOffset[1]) *
            ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;

    UTILS_LP_FAST(m->v_bus, GET_INPUT_VOLTAGE(), 0.1f);
}

/*
 * DMA 传输完成中断回调 (ADC 采样由 DMA 搬运完成后触发)。
 *   分频判断 → 参数采样换算 → 开环 FOC 运算。
 * 这是整个 FOC 控制链的驱动入口，由 ADC-DMA-TIM 联动触发。
 */
void Motor_DmaInt(void *p, uint32_t flags)
{
    float dt;

    (void)p;
    (void)flags;

    if (!Motor_DmaInt_ShouldRun()) {
        return;
    }

    Motor_It_ParamCal();
    dt = Motor_OpenLoop_GetDt(&motor);
    /* 保护位于采样换算之后、FOC 输出之前；故障后清 PI 并保持安全电平。 */
    if (Motor_Protect_Run(&motor, dt)) {
        Motor_OpenLoop_ResetRuntime(&motor);
        Motor_OpenLoop_OutputSafe(&motor);
        return;
    }
    Motor_OpenLoop_Run(&motor, motor.v_bus);
}

#ifdef __cplusplus
}
#endif
