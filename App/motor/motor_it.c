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
#include "motor_alignment.h"
#include "motor_hfi_start.h"
#include "motor_if_start.h"

/* 电流零点正常应在 ADC 中点附近。flash 没校准或读出异常时，用默认值兜底。 */
#define MOTOR_CURRENT_OFFSET_MIN      (1000.0f)
#define MOTOR_CURRENT_OFFSET_MAX      (3000.0f)
#define MOTOR_CURRENT_OFFSET_DEFAULT  (2045.0f)

/* DQ 轴电压中间量，用于电流环 PI 输出与限幅之间的传递。 */
typedef struct {
    float vd; /* D 轴电压 (励磁分量) */
    float vq; /* Q 轴电压 (转矩分量) */
} motor_dq_voltage_t;

static motor_hfi_start_t s_hfi_start;
static void Motor_Foc_OutputVoltage(MOTOR_PARAM_S *m, float vd, float vq,
                                    float theta, float v_bus);

/* I/F 启动算法实例；算法本身位于可独立移植的 motor_if_start 模块。 */
static motor_if_start_t s_if_start = {
    .first_cycle = true,
    .iq_scale = 1.0f
};

static motor_alignment_t s_alignment = {
    .stopped = true
};

/*
 * ADC-DMA 中断执行时间统计。
 * TIM5 在 BSP 中配置为 1MHz，所以计数差值直接等于微秒数。分频为 2 时中断
 * 路径交替为“采样+保护”和“采样+保护+完整 FOC”，平均值因此可以直接乘以
 * 30000Hz 回调频率估算总 CPU 占用；最大值用于观察完整 FOC 的最慢一次。
 */
static volatile float s_foc_runtime_average_us = 0.0f;
static volatile float s_foc_runtime_maximum_us = 0.0f;

static void Motor_DmaInt_RecordRuntime(uint32_t start_time)
{
    uint32_t elapsed_us = Bsp_Tim_DtNow() - start_time;
    float elapsed = (float)elapsed_us;

    /* 调试器暂停或 TIM5 尚未初始化时可能得到异常大值，不把它污染到统计中。 */
    if (elapsed_us > 1000U) {
        return;
    }

    if (s_foc_runtime_average_us <= 0.0f) {
        s_foc_runtime_average_us = elapsed;
    } else {
        s_foc_runtime_average_us +=
            0.02f * (elapsed - s_foc_runtime_average_us);
    }

    if (elapsed > s_foc_runtime_maximum_us) {
        s_foc_runtime_maximum_us = elapsed;
    }
}

/*
 * 计算 ADC-DMA 相邻两次回调的基础周期。
 * TIM1 是中心对齐 PWM，计数顶部和底部各产生一次触发，所以
 * SYSTEM_CORE_CLOCK / ARR 正好是 30000Hz ADC-DMA 回调频率。
 */
static float Motor_OpenLoop_GetAdcDt(MOTOR_PARAM_S *m)
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

    return 1.0f / foc_freq;
}

/* 完整 FOC 只在分频计数满足时运行，因此它的 dt 等于基础 ADC 周期乘分频数。 */
static float Motor_OpenLoop_GetDt(MOTOR_PARAM_S *m)
{
    return Motor_OpenLoop_GetAdcDt(m) *
           (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
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
    Motor_Sensorless_SetAppliedVoltage(0.0f, 0.0f);
}

/*
 * 复位开环运行时状态：
 *   - 当前转速清零
 *   - D/Q 电流显示值和 PI 积分项清零
 *   - 预对齐状态清零，停机标志置位 (下次启动重新预对齐)
 */
static void Motor_OpenLoop_ResetRuntime(MOTOR_PARAM_S *m)
{
    m->ol_current_rpm = 0.0f;
    m->Id = 0.0f;
    m->Iq = 0.0f;
    m->ol_vd_int = 0.0f;
    m->ol_vq_int = 0.0f;
    Motor_SpeedLoop_Reset();
    Motor_Sensorless_Reset();
    MotorAlignment_Reset(&s_alignment);
    MotorIfStart_Reset(&s_if_start);
    MotorHfiStart_Reset(&s_hfi_start);
}

/*
 * 停止当前开环输出：
 *   复位运行时状态 → 输出安全电平。
 *
 * 这里不能修改 ol_target_rpm。目标转速属于任务/Shell 下发的命令，
 * 如果高频 DMA 中断在“写目标转速”和“置位 ol_enable”之间抢占任务，
 * 清零目标会导致启动命令偶发丢失。需要撤销运行命令时，由 olstop、
 * 故障清除或偏移校准同时清除 ol_enable 和 ol_target_rpm；故障锁存
 * 本身则会在不改写用户命令的情况下持续禁止 PWM 控制输出。
 */
static void Motor_OpenLoop_Stop(MOTOR_PARAM_S *m)
{
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

    /* 对齐电流/时间为 0 时视为关闭预对齐，保留旧的一拍相位补偿作为回退。 */
    {
        motor_alignment_config_t alignment_config = {
            m->ol_align_enable, m->ol_align_current,
            m->ol_align_time, m->ol_align_angle
        };
        MotorAlignment_NormalizeConfig(&alignment_config);
        m->ol_align_current = alignment_config.current;
        m->ol_align_time = alignment_config.duration;
    }
    utils_norm_angle_rad(&m->ol_align_angle);
    {
        motor_hfi_start_config_t hfi_config = {
            m->hfi_start_enable, m->hfi_start_voltage,
            m->hfi_start_cycles, m->hfi_start_min_confidence, 0.20f
        };
        MotorHfiStart_NormalizeConfig(&hfi_config);
        m->hfi_start_voltage = hfi_config.voltage;
        m->hfi_start_cycles = hfi_config.cycles_per_axis;
        m->hfi_start_min_confidence = hfi_config.min_confidence;
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

    /* 速度环参数防御性钳位，避免 Flash 异常值导致除零或输出方向异常。 */
    if (m->speed_kp < 0.0f) {
        m->speed_kp = 0.0f;
    }
    if (m->speed_ki < 0.0f) {
        m->speed_ki = 0.0f;
    }
    if (m->speed_iq_limit < 0.0f) {
        m->speed_iq_limit = -m->speed_iq_limit;
    }
    if (m->speed_loop_hz < 1.0f) {
        m->speed_loop_hz = 1.0f;
    }
    if (m->sensorless_motor_r < 0.0f) m->sensorless_motor_r = 0.0f;
    if (m->sensorless_motor_l < 0.0f) m->sensorless_motor_l = 0.0f;
    if (m->sensorless_flux_linkage < 1.0e-6f) m->sensorless_flux_linkage = 1.0e-6f;
    if (m->sensorless_observer_gain < 0.0f) m->sensorless_observer_gain = 0.0f;
    if (m->sensorless_pll_kp < 0.0f) m->sensorless_pll_kp = 0.0f;
    if (m->sensorless_pll_ki < 0.0f) m->sensorless_pll_ki = 0.0f;
    if (m->sensorless_transition_rpm < 0.0f) m->sensorless_transition_rpm = 0.0f;
    if (m->sensorless_fallback_rpm < 0.0f) m->sensorless_fallback_rpm = 0.0f;
    if (m->sensorless_fallback_rpm > m->sensorless_transition_rpm) {
        m->sensorless_fallback_rpm = m->sensorless_transition_rpm;
    }
    if (m->sensorless_openloop_max_rpm < 0.0f) {
        m->sensorless_openloop_max_rpm = -m->sensorless_openloop_max_rpm;
    }
    if (m->sensorless_openloop_max_rpm > 0.0f &&
        m->sensorless_openloop_max_rpm < m->sensorless_transition_rpm) {
        m->sensorless_openloop_max_rpm = m->sensorless_transition_rpm;
    }
    if (m->sensorless_openloop_max_rpm > m->ol_max_rpm) {
        m->sensorless_openloop_max_rpm = m->ol_max_rpm;
    }
    if (m->sensorless_lock_time < 0.0f) m->sensorless_lock_time = 0.0f;
    if (m->sensorless_loss_time < 0.0f) m->sensorless_loss_time = 0.0f;
    if (m->sensorless_blend_time < 0.0f) m->sensorless_blend_time = 0.0f;
    if (m->sensorless_phase_tolerance < 0.0f) m->sensorless_phase_tolerance = 0.0f;
    /* 接管相位差必须小于 90°，否则电流坐标可能跨过最大转矩点。 */
    if (m->sensorless_phase_tolerance > 1.50f) m->sensorless_phase_tolerance = 1.50f;
    if (m->sensorless_speed_filter < 0.0f) m->sensorless_speed_filter = 0.0f;
    if (m->sensorless_speed_filter > 1.0f) m->sensorless_speed_filter = 1.0f;
}

/*
 * 在 0..PI 上扫描凸极轴。每拍交替施加正/负 d 轴电压，下一拍读取该轴电流；
 * 电流增量最大的方向对应较小电感轴。正负响应的不对称用于区分磁极极性。
 * 表贴式电机或采样噪声过大时凸极度不足，可信度检查会让流程回退到固定 Id 对齐。
 */
static bool Motor_HfiStart_Run(MOTOR_PARAM_S *m, float v_bus)
{
    motor_hfi_start_config_t config = {
        m->hfi_start_enable, m->hfi_start_voltage, m->hfi_start_cycles,
        m->hfi_start_min_confidence, 0.20f
    };
    motor_hfi_start_output_t output;
    motor_hfi_start_result_t result;
    bool starting;

    if (!config.enabled || config.voltage <= 0.0f || s_hfi_start.attempted) {
        return false;
    }
    starting = !s_hfi_start.running;

    if (starting) {
        m->ol_vd_int = 0.0f;
        m->ol_vq_int = 0.0f;
        Motor_SpeedLoop_Reset();
        Motor_Sensorless_Reset();
    }

    clarke_transform(m->ia, m->ib, m->ic, &m->Ialpha, &m->Ibeta);
    result = MotorHfiStart_Step(&s_hfi_start, &config, m->Ialpha, m->Ibeta,
                                v_bus, &output);
    if (result == MOTOR_HFI_START_RUNNING) {
        Motor_Foc_OutputVoltage(m, output.vd, 0.0f, output.theta, v_bus);
        return true;
    }
    if (result == MOTOR_HFI_START_FINISHED) {
        Motor_OpenLoop_OutputSafe(m);
        if (s_hfi_start.succeeded) {
            m->ol_theta = s_hfi_start.angle;
            MotorAlignment_MarkComplete(&s_alignment);
            MotorIfStart_SetNeutral(&s_if_start);
        }
    }
    return false;
}

/*
 * 角度积分转速整理。
 * 每个 FOC 周期在 Startup 之后调用一次，统一整理本拍派生状态。
 *
 * 处理内容：
 *   1. 已预对齐的启动：直接使用软启动斜坡的当前转速。转子位置已知，
 *      必须从 0RPM 平滑推进角度，不能立刻跳到 30RPM。
 *   2. 未预对齐的兼容旧流程：保留最低转速跳转，防止磁场冻住。
 *
 * 注意：相位补偿和 Iq 缩放已移到 Motor_OpenLoop_StartupIf 中处理，
 * 本函数只负责角度积分转速，由 UpdateTheta 消费。
 */
/*
 * V/F 模式起步处理：
 *   目标转速超过 ol_min_rpm 时，直接将当前转速跳到 ol_min_rpm，
 *   避免低速段电压不足无法起转，然后把实际转速交给启动模块用于角度积分。
 *   V/F 模式不需要相位补偿和 Iq 缩放，直接置为默认值。
 */
static void Motor_OpenLoop_StartupVf(MOTOR_PARAM_S *m)
{
    float target = m->ol_target_rpm;
    float rpm = m->ol_current_rpm;

    if (fabsf(rpm) < m->ol_min_rpm &&
        fabsf(target) >= m->ol_min_rpm) {
        m->ol_current_rpm = SIGN(target) * m->ol_min_rpm;
    }

    MotorIfStart_SetVfRpm(&s_if_start, m->ol_current_rpm);
}

/*
 * I/F 启动前的转子预对齐。
 *
 * 无霍尔、无编码器时，刚上电并不知道转子磁钢实际停在哪个角度。如果直接
 * 积分 I/F 角度，定子磁场会马上旋转，转子往往只来回摆动，随后失步过流。
 *
 * 本函数在首次启动时把 ol_theta 固定在 ol_align_angle，电流目标由
 * Motor_CurrentTarget_Get() 切换为“Id=ol_align_current、Iq=0”。恒定 D 轴
 * 磁场会把转子吸到这个已知角度。保持 ol_align_time 后再返回 false，让
 * 调用者进入正常 I/F 转速斜坡。
 *
 * 返回 true：仍在对齐，禁止更新转速/角度和无感观测器；
 * 返回 false：可以执行正常 I/F 启动。
 */
static bool Motor_OpenLoop_RunAlignment(MOTOR_PARAM_S *m, float dt)
{
    motor_alignment_config_t config = {
        m->ol_align_enable, m->ol_align_current,
        m->ol_align_time, m->ol_align_angle
    };
    motor_alignment_output_t output;
    motor_alignment_result_t result =
        MotorAlignment_Step(&s_alignment, &config, dt, &output);

    if (result == MOTOR_ALIGNMENT_STARTED) {
        MotorIfStart_SetNeutral(&s_if_start);
        m->ol_current_rpm = 0.0f;
        m->ol_vd_int = 0.0f;
        m->ol_vq_int = 0.0f;
        Motor_SpeedLoop_Reset();
        Motor_Sensorless_Reset();
    }

    if (result == MOTOR_ALIGNMENT_STARTED ||
        result == MOTOR_ALIGNMENT_RUNNING) {
        m->ol_theta = output.angle;
        m->ol_current_rpm = 0.0f;
        MotorIfStart_SetNeutral(&s_if_start);
        return true;
    }

    if (result == MOTOR_ALIGNMENT_FINISHED) {
        m->ol_vd_int = 0.0f;
        m->ol_vq_int = 0.0f;
        Motor_SpeedLoop_Reset();
        Motor_Sensorless_Reset();
    }
    return false;
}

/*
 * 正常 I/F 加速阶段的启动处理。
 *
 * 预对齐关闭时，为兼容旧行为保留“一拍 phase_boost”。预对齐开启时，
 * RunAlignment 已经把 was_stopped 清掉，因此这里不会再额外注入 90° 跳变。
 * 随后由 motor_if_start 模块计算低速 Iq 比例和角度积分转速。
 */
static void Motor_OpenLoop_StartupIf(MOTOR_PARAM_S *m)
{
    motor_if_start_config_t config = {
        m->ol_min_rpm, m->ol_start_phase_boost, 0.1f
    };
    motor_if_start_input_t input = {
        m->ol_target_rpm, m->ol_current_rpm, s_alignment.complete
    };

    MotorIfStart_Step(&s_if_start, &config, &input);
    if (!s_alignment.complete && !s_if_start.first_cycle) {
        MotorAlignment_MarkSkipped(&s_alignment);
    }
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
     * 无感尚未 CLOSED 时，I/F 只是启动手段，不能无限强拖到目标高速。
     * 默认最多走到 800RPM 并保持，等待观测器四项条件锁定且角度渐变完成；
     * 只有 100% 使用观测角后才继续向 2000RPM 加速。将上限设为 0 可显式
     * 关闭此保护，但调试阶段不建议这样做。
     */
    if (m->ol_mode == MOTOR_OL_MODE_IF &&
        !Motor_Sensorless_IsTakeoverComplete() &&
        m->sensorless_openloop_max_rpm > 0.0f &&
        fabsf(target) > m->sensorless_openloop_max_rpm) {
        target = SIGN(target) * m->sensorless_openloop_max_rpm;
    }

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
    m->ol_theta += RPM2RADPS_f(s_if_start.angle_rpm) * m->ol_pole_pairs * dt;
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
    /* 观测器使用本次下发电压，在下一 FOC 周期积分定子磁链。 */
    Motor_Sensorless_SetAppliedVoltage(v_alpha, v_beta);
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
    float theta = m->ol_theta + s_if_start.phase_boost;

    utils_norm_angle_rad(&theta);
    return Motor_Sensorless_GetControlAngle(theta);
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
 *   - 预对齐阶段：固定 Id=ol_align_current、Iq=0，产生不旋转的定子磁场；
 *   - 正常 I/F 阶段：Id 取 ol_id_set，Iq 取 ol_iq_set 并做低速缩放；
 *   - 无感闭环 + 速度环：速度 PI 输出替代手动 Iq。
 */
static void Motor_CurrentTarget_Get(MOTOR_PARAM_S *m, float *id_target,
                                    float *iq_target)
{
    if (s_alignment.running) {
        /*
         * 对齐必须使用 D 轴电流而不是 Q 轴电流。Q 轴电流负责转矩，会让
         * 转子继续摆动；D 轴磁场固定后，转子会自行吸到已知电角位置。
         */
        *id_target = m->ol_align_current;
        *iq_target = 0.0f;
        return;
    }

    *id_target = m->ol_id_set;
    if (m->speed_enable && Motor_SpeedLoop_IsFeedbackValid()) {
        /* 无感闭环建立后，速度 PI 的输出作为电流内环 Iq 目标。 */
        *iq_target = Motor_SpeedLoop_GetIqTarget();
    } else {
        /* 观测器锁定前必须保留 I/F 启动电流，否则无感电机无法从零速起转。 */
        *iq_target = m->ol_iq_set;
        *iq_target *= s_if_start.iq_scale;
    }
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
 *   1. 欠压 → 复位 + 安全输出；
 *   2. 未就绪 → 停机；
 *   3. I/F 模式先执行固定 D 轴预对齐，再进入转速斜坡与无感观测；
 *   4. V/F 模式维持原有启动流程。
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

    Motor_OpenLoop_NormalizeConfig(m);
    if (m->ol_mode == MOTOR_OL_MODE_IF) {
        if (Motor_HfiStart_Run(m, v_bus)) {
            return;
        }
        if (!Motor_OpenLoop_RunAlignment(m, dt)) {
            /* 对齐结束后才允许转速斜坡和角度积分开始旋转磁场。 */
            Motor_OpenLoop_UpdateSpeed(m, dt);
            Motor_OpenLoop_StartupIf(m);
            Motor_OpenLoop_UpdateTheta(m, dt);
            /* 观测器后台运行；满足锁定条件后渐变接管角度并发布真实转速。 */
            Motor_Sensorless_Update(m, dt);
            /* 速度外环先更新 Iq 目标，随后本周期电流内环读取该目标。 */
            Motor_SpeedLoop_Run(m, dt);
        }
        Motor_OpenLoop_RunIf(m, v_bus, dt);
    } else {
        Motor_SpeedLoop_Reset();
        Motor_Sensorless_Reset();
        Motor_OpenLoop_StartupVf(m);
        Motor_OpenLoop_UpdateTheta(m, dt);
        Motor_OpenLoop_RunVf(m, v_bus);
    }
}

/**
 * @description: 查询当前是否正处于 I/F 启动前的转子预对齐阶段。
 * @return {*}
 * @author: lkc
 */
bool Motor_OpenLoop_IsAligning(void)
{
    return s_alignment.running;
}

/**
 * @description: 返回本次转子预对齐已进行的时间 (s)，用于状态显示与调试。
 * @return {*}
 * @author: lkc
 */
float Motor_OpenLoop_GetAlignElapsed(void)
{
    return s_alignment.elapsed;
}

/**
 * @description: 查询 HFI 凸极扫描启动流程是否正在运行。
 * @return {*}
 * @author: lkc
 */
bool Motor_HfiStart_IsRunning(void) { return s_hfi_start.running; }
/**
 * @description: 查询 HFI 凸极扫描是否已成功识别出转子初始电角度。
 * @return {*}
 * @author: lkc
 */
bool Motor_HfiStart_Succeeded(void) { return s_hfi_start.succeeded; }
/**
 * @description: 返回 HFI 凸极扫描识别出的转子电角度 (rad)。
 * @return {*}
 * @author: lkc
 */
float Motor_HfiStart_GetAngle(void) { return s_hfi_start.angle; }
/**
 * @description: 返回 HFI 凸极扫描识别的可信度，用于判断识别质量。
 * @return {*}
 * @author: lkc
 */
float Motor_HfiStart_GetConfidence(void) { return s_hfi_start.confidence; }

/**
 * @description: 返回 ADC-DMA 中断的平均执行时间。
 * @detail: 单位为 us；统计覆盖“采样+保护”和完整 FOC 两类回调，结合 ADC 频率可估算总 CPU 占用。
 * @return {*}
 * @author: lkc
 */
float Motor_DmaInt_GetAverageRuntimeUs(void)
{
    return s_foc_runtime_average_us;
}

/**
 * @description: 返回 ADC-DMA 中断的历史最长执行时间。
 * @detail: 单位为 us，用于观察完整 FOC 最慢一次的耗时。
 * @return {*}
 * @author: lkc
 */
float Motor_DmaInt_GetMaximumRuntimeUs(void)
{
    return s_foc_runtime_maximum_us;
}

/**
 * @description: 返回实际 ADC-DMA 触发频率。
 * @detail: 由 TIM1 中心对齐 PWM 配置实时换算（顶部/底部各触发一次，正常约 30kHz）；TIM1 未初始化时回退到用户配置的 ol_foc_freq。
 * @return {*}
 * @author: lkc
 */
float Motor_DmaInt_GetAdcFrequencyHz(void)
{
    uint32_t arr = TIM1->ARR;
    uint32_t psc = TIM1->PSC + 1U;
    float adc_trigger_hz;

    if (arr > 0U && psc > 0U) {
        /* 中心对齐 PWM 在计数顶部和底部各产生一次 ADC 触发。 */
        adc_trigger_hz = (float)SYSTEM_CORE_CLOCK /
                         ((float)arr * (float)psc);
    } else {
        adc_trigger_hz = motor.ol_foc_freq;
    }

    return adc_trigger_hz;
}

/**
 * @description: 返回 FOC 控制环实际运行频率。
 * @detail: 等于 ADC 触发频率除以分频数 FOC_CONTROL_LOOP_FREQ_DIVIDER。
 * @return {*}
 * @author: lkc
 */
float Motor_DmaInt_GetControlFrequencyHz(void)
{
    return Motor_DmaInt_GetAdcFrequencyHz() /
           (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
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

/**
 * @description: 中断内采样三相电流/相电压并换算为物理量。
 * @detail: 每个 ADC-DMA 回调执行一次：电流减零点偏移后乘系数得 ia/ib/ic (A)；
 *          相电压减未驱动偏移并乘分压系数得 va/vb/vc (V)；母线电压经低通滤波得 v_bus (V)。
 *          vb/vc 的 ADC 通道与相序非一一对应，见硬件映射。
 * @return {*}
 * @author: lkc
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
            PHASE_VOLTAGE_DIVIDER_GAIN * ADC_VOLTS_PH_FACTOR;
    m->vb = (ADC_VOLTS(ADC_IND_SENS3) - m->voltundrivenOffset[2]) *
            PHASE_VOLTAGE_DIVIDER_GAIN * ADC_VOLTS_PH_FACTOR;
    m->vc = (ADC_VOLTS(ADC_IND_SENS2) - m->voltundrivenOffset[1]) *
            PHASE_VOLTAGE_DIVIDER_GAIN * ADC_VOLTS_PH_FACTOR;

    UTILS_LP_FAST(m->v_bus, GET_INPUT_VOLTAGE(), 0.1f);
}

/*
 * 正常 FOC 模式下的 ISR 路径。
 * ParamCal + 30kHz 过流保护 + 分频后的开环 FOC 全流程。
 * 由 Motor_DmaInt 调度，不直接注册到中断向量表。
 */
static void Motor_DmaInt_Foc(void)
{
    uint32_t runtime_start = Bsp_Tim_DtNow();
    float protection_dt;

    /*
     * 采样换算和保护每个 30kHz ADC 回调都执行，不受 FOC 分频影响。
     * 这样软件绝对过流、欠压等保护仍保持约 33us 的检查间隔。
     */
    Motor_It_ParamCal();
    protection_dt = Motor_OpenLoop_GetAdcDt(&motor);

    /* 保护位于采样换算之后、FOC 输出之前；故障后清 PI 并保持安全电平。 */
    if (Motor_Protect_Run(&motor, protection_dt)) {
        Motor_OpenLoop_ResetRuntime(&motor);
        Motor_OpenLoop_OutputSafe(&motor);
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }

    /* 只有电流环、无感观测器、速度环和 SVPWM 使用分频。 */
    if (!Motor_DmaInt_ShouldRun()) {
        Motor_DmaInt_RecordRuntime(runtime_start);
        return;
    }

    Motor_OpenLoop_Run(&motor, motor.v_bus);

    /*
     * 无感观测器可能在本次 FOC 中确认”高速失锁”并直接锁存故障。
     * 再检查一次可在退出本次 DMA 中断前覆盖为安全占空比，而不是等到
     * 下一次 ADC 回调；普通过流/欠压仍由上面的 30kHz 保护入口判断。
     */
    if (Motor_Protect_IsFaultLatched()) {
        Motor_OpenLoop_ResetRuntime(&motor);
        Motor_OpenLoop_OutputSafe(&motor);
    }
    Motor_DmaInt_RecordRuntime(runtime_start);
}

/**
 * @description: ADC-DMA 传输完成中断回调，FOC 控制链驱动入口。
 * @detail: 由 ADC-DMA-TIM 联动触发；参数检测占用功率桥时直接返回，否则进入 FOC 路径。
 * @param {void} *p     回调上下文，未使用
 * @param {uint32_t} flags  中断标志，未使用
 * @return {*}
 * @author: lkc
 */
void Motor_DmaInt(void *p, uint32_t flags)
{
    (void)p;
    (void)flags;

    if (Motor_Detect_IsActive()) {
        return;
    }

    Motor_DmaInt_Foc();
}

#ifdef __cplusplus
}
#endif
