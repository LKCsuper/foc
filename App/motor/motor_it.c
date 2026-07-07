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

#define MOTOR_OL_MODE_VF              (0)
#define MOTOR_OL_MODE_CURRENT         (1)

/* 电流零点正常应在 ADC 中点附近。flash 没校准或读出异常时，用默认值兜底。 */
#define MOTOR_CURRENT_OFFSET_MIN      (1000.0f)
#define MOTOR_CURRENT_OFFSET_MAX      (3000.0f)
#define MOTOR_CURRENT_OFFSET_DEFAULT  (2045.0f)

/* DQ 轴电压中间量，用于电流环 PI 输出与限幅之间的传递。 */
typedef struct {
    float vd; /* D 轴电压 (励磁分量) */
    float vq; /* Q 轴电压 (转矩分量) */
} motor_dq_voltage_t;

/* 启动相位补偿只在“停机 -> 运行”的第一拍生效，避免运行中反复跳相位。 */
static bool s_ol_was_stopped = true;

/*
 * 计算开环 FOC 的控制周期 dt。
 * dt = FOC_CONTROL_LOOP_FREQ_DIVIDER / ol_foc_freq
 * ol_foc_freq 为用户设定的 FOC 执行频率，最小钳位到 1Hz。
 */
static float Motor_OpenLoop_GetDt(MOTOR_PARAM_S *m)
{
    if (m->ol_foc_freq <= 1.0f) {
        m->ol_foc_freq = 1.0f;
    }

    return (float)FOC_CONTROL_LOOP_FREQ_DIVIDER / m->ol_foc_freq;
}

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
    s_ol_was_stopped = true;
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
 *   2) 目标转速 > 0
 *   3) 母线电压 >= 1V (避免除零和异常输出)
 */
static bool Motor_OpenLoop_IsReady(MOTOR_PARAM_S *m, float v_bus)
{
    return m->ol_enable && m->ol_target_rpm > 0.0f && v_bus >= 1.0f;
}

/*
 * 对用户配置参数做合法性钳位：
 *   - 极对数 >= 1
 *   - 最低转速/爬坡速率 >= 0
 *   - 最高转速 >= 1
 *   - 目标转速不超过最高转速
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
}

static void Motor_OpenLoop_UpdateSpeed(MOTOR_PARAM_S *m, float dt)
{
    float target = m->ol_target_rpm;
    float step = m->ol_ramp_rpm_s * dt;

    /*
     * V/F 模式从最低转速起步更容易拉起来。
     * 电流模式保留从 0 爬升，这样低速 Iq 缩放才有意义。
     */
    if (m->ol_mode != MOTOR_OL_MODE_CURRENT &&
        m->ol_current_rpm < m->ol_min_rpm &&
        target > 0.0f) {
        m->ol_current_rpm = m->ol_min_rpm;
    }

    /*
     * ramp 为 0 时直接跳到目标转速。
     * 否则按 rpm/s 斜坡逼近目标，避免一步越过后又反向修正。
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

static void Motor_OpenLoop_UpdateTheta(MOTOR_PARAM_S *m, float dt)
{
    float rpm_for_angle = m->ol_current_rpm;

    /*
     * 角度不能完全停在 0，否则开环启动时磁场不转，只会在固定方向上吸住。
     * 所以低于 ol_min_rpm 时仍按最低转速积分角度。
     */
    if (rpm_for_angle < m->ol_min_rpm) {
        rpm_for_angle = m->ol_min_rpm;
    }

    /* 机械转速 -> 机械角速度 -> 电角速度，再按 dt 积分成电角度。 */
    m->ol_theta += RPM2RADPS_f(rpm_for_angle) * m->ol_pole_pairs * dt;
    utils_norm_angle_rad(&m->ol_theta);
}

/*
 * 每控制周期更新运动状态：
 *   参数钳位 → 转速斜坡跟踪 → 电角度积分。
 */
static void Motor_OpenLoop_UpdateMotion(MOTOR_PARAM_S *m, float dt)
{
    Motor_OpenLoop_NormalizeConfig(m);
    Motor_OpenLoop_UpdateSpeed(m, dt);
    Motor_OpenLoop_UpdateTheta(m, dt);
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
    float f_elec = m->ol_current_rpm / 60.0f * m->ol_pole_pairs;
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

/* 获取电流环使用的电角度，启动第一拍可叠加一次相位补偿。 */
static float Motor_CurrentLoop_GetTheta(MOTOR_PARAM_S *m)
{
    float theta = m->ol_theta;

    /*
     * 电流模式刚启动时给一次相位提前，等价于先把转矩方向摆出来。
     * 只补一次，后续完全跟随软件积分角度。
     */
    if (s_ol_was_stopped && m->ol_current_rpm > 0.0f) {
        theta += m->ol_start_phase_boost;
        s_ol_was_stopped = false;
    }

    if (m->ol_current_rpm <= 0.0f) {
        s_ol_was_stopped = true;
    }

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

static void Motor_CurrentTarget_Get(MOTOR_PARAM_S *m, float *id_target,
                                    float *iq_target)
{
    *id_target = m->ol_id_set;
    *iq_target = m->ol_iq_set;

    /*
     * 起步低于最低转速时，先按转速比例收小 Iq。
     * 这样电机还没转起来时不会一上来就给满转矩电流。
     */
    if (m->ol_current_rpm < m->ol_min_rpm && m->ol_min_rpm > 0.0f) {
        float scale = m->ol_current_rpm / m->ol_min_rpm;

        if (scale < 0.1f) {
            scale = 0.1f;
        }

        *iq_target *= scale;
    }
}

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

static void Motor_OpenLoop_RunCurrent(MOTOR_PARAM_S *m, float v_bus, float dt)
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

    /*
     * 命令层可能在两个中断之间完成 olstop -> olstart。
     * 只要当前转速已经清零，就把下一次电流模式启动当作新的启动过程。
     */
    if (m->ol_current_rpm <= 0.0f) {
        s_ol_was_stopped = true;
    }

    dt = Motor_OpenLoop_GetDt(m);
    Motor_OpenLoop_UpdateMotion(m, dt);

    if (m->ol_mode == MOTOR_OL_MODE_CURRENT) {
        Motor_OpenLoop_RunCurrent(m, v_bus, dt);
    } else {
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
    (void)p;
    (void)flags;

    if (!Motor_DmaInt_ShouldRun()) {
        return;
    }

    Motor_It_ParamCal();
    Motor_OpenLoop_Run(&motor, motor.v_bus);
}

#ifdef __cplusplus
}
#endif
