#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"

/*
 * ======================== 无感观测器整体流程 ========================
 *
 * 本文件解决的问题是：没有霍尔和编码器时，如何估算转子的电角度和转速。
 * 当前使用“Ortega 磁链观测器 + PLL 锁相环”，每个 FOC 周期执行一次：
 *
 *   上一周期实际下发的 Vα/Vβ + 本周期采样的 Ia/Ib/Ic
 *                             │
 *                             ▼ Clarke 变换
 *                          Iα / Iβ
 *                             │
 *                             ▼ 电机模型积分
 *                 定子磁链 xα / xβ = ∫(V - R×I + 修正项)dt
 *                             │
 *                             ▼ 减去电感磁链 L×I
 *                 转子磁链 ψα / ψβ = x - L×I
 *                             │
 *                             ▼ atan2
 *                        原始观测电角度
 *                             │
 *                             ▼ PLL 滤波和跟踪
 *                    平滑电角度 + 电角速度
 *                             │
 *                             ▼ 除以极对数
 *                          机械转速 RPM
 *
 * 重要限制：电机静止或转速很低时，反电动势信息太弱，观测结果不可信。
 * 所以系统必须先用 I/F 强制角度让电机转起来，满足磁链、转速、方向、
 * 相位和持续时间条件后，才允许观测角度渐变接管电流环。
 *
 * 本文件只负责“估算与接管判断”。真正的 D/Q 电流 PI 和 SVPWM 仍在
 * motor_it.c 中执行，速度 PI 则在 motor_speed.c 中执行。
 */

/*
 * Ortega 观测器、PLL 和接管状态的运行时数据。
 * 这里的角度全部是“电角度”，单位 rad；除特别说明外，时间单位都是 s。
 */
static struct {
    float x_alpha;                /* α 轴估算定子磁链，单位 Wb。 */
    float x_beta;                 /* β 轴估算定子磁链，单位 Wb。 */
    float applied_v_alpha;        /* 上一周期实际下发的 α 轴电压，单位 V。 */
    float applied_v_beta;         /* 上一周期实际下发的 β 轴电压，单位 V。 */
    float observer_angle;         /* atan2 得到的原始观测电角度，单位 rad。 */
    float pll_angle;              /* PLL 平滑后的观测电角度，单位 rad。 */
    float pll_speed_electrical;   /* PLL 估算电角速度，单位 electrical rad/s。 */
    float mechanical_rpm;         /* PLL 电角速度直接换算的机械 RPM。 */
    float filtered_rpm;           /* 一阶低通后的机械 RPM，供状态判断和速度环使用。 */
    float flux_magnitude;         /* 转子磁链矢量模长，单位 Wb。 */
    float phase_error;            /* PLL 角度与 I/F 强制角度之差，单位 rad。 */
    float lock_timer;             /* 所有锁定条件连续满足的累计时间。 */
    float loss_timer;             /* 闭环后失锁条件连续满足的累计时间。 */
    float blend;                  /* 角度接管比例：0=全开环，1=全观测角。 */
    float control_angle;          /* 最终送入 Park 变换的电角度，单位 rad。 */
    bool flux_ok;                 /* 磁链接管条件。 */
    bool speed_ok;                /* 转速接管条件。 */
    bool direction_ok;            /* 方向接管条件。 */
    bool phase_ok;                /* 相位接管条件。 */
    bool initialized;             /* false 表示磁链积分器还没有用开环角度初始化。 */
    motor_sensorless_state_t state; /* OPEN_LOOP、LOCKING 或 CLOSED。 */
} s_sensorless;

/* 将 value 限制到 [minimum, maximum]，防止速度、积分量或混合比例越界。 */
static float Motor_Sensorless_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

/*
 * 使用当前 I/F 强制角度初始化磁链积分器。
 *
 * PMSM 的简化磁链关系是：
 *
 *   定子磁链 ψs = L × Is + 转子永磁体磁链 ψf
 *
 * 已知当前电流 Iα/Iβ，并暂时把 I/F 的 ol_theta 当作转子方向，就可以
 * 构造一个“模长等于额定磁链、方向等于开环角度”的初始转子磁链。
 * 这样 x_alpha/x_beta 不会从全零开始，第一次 atan2 也不会得到随机角度。
 *
 * 注意：这只是数值初始化，不代表开环角度一定是真实转子角度。后续仍需
 * 依靠电机转动产生的电压/电流信息，让观测器逐渐收敛。
 */
static void Motor_Sensorless_Seed(const MOTOR_PARAM_S *motor,
                                  float i_alpha, float i_beta)
{
    float inductance = motor->sensorless_motor_l;
    float flux = motor->sensorless_flux_linkage;

    /* L×I 是定子电感磁链，flux×cos/sin 是假定的永磁体磁链。 */
    s_sensorless.x_alpha = inductance * i_alpha + flux * cosf(motor->ol_theta);
    s_sensorless.x_beta = inductance * i_beta + flux * sinf(motor->ol_theta);

    /* 原始角、PLL 角和控制角从同一角度出发，避免启动瞬间发生角度阶跃。 */
    s_sensorless.observer_angle = motor->ol_theta;
    s_sensorless.pll_angle = motor->ol_theta;
    s_sensorless.pll_speed_electrical = 0.0f;
    s_sensorless.control_angle = motor->ol_theta;
    s_sensorless.initialized = true;
}

/*
 * 使用二阶 PLL 跟踪原始观测角度。
 *
 *   delta = observer_angle - pll_angle          （最短角度差，范围约 ±π）
 *   d(pll_angle)/dt = pll_speed + Kp × delta
 *   d(pll_speed)/dt = Ki × delta
 *
 * Kp 负责让 PLL 角度快速追上观测角，Ki 的积分结果就是估算电角速度。
 * PLL 可以滤掉 atan2 角度中的高频抖动，并连续输出带正负方向的速度。
 * 输出速度单位是 electrical rad/s，后续除以极对数换算机械 RPM。
 */
static void Motor_Sensorless_RunPll(const MOTOR_PARAM_S *motor, float dt)
{
    float delta = utils_angle_difference_rad(s_sensorless.observer_angle,
                                             s_sensorless.pll_angle);
    float max_electrical_speed;

    /* 先用“当前速度 + 比例相位校正”推进 PLL 角度。 */
    s_sensorless.pll_angle +=
        (s_sensorless.pll_speed_electrical + motor->sensorless_pll_kp * delta) * dt;
    utils_norm_angle_rad(&s_sensorless.pll_angle);

    /* 再积分相位误差，得到能够长期跟踪转速的电角速度。 */
    s_sensorless.pll_speed_electrical += motor->sensorless_pll_ki * delta * dt;

    /*
     * 将 PLL 速度限制在“最高机械转速 × 极对数 × 1.5”以内。
     * 1.5 倍留出加速余量，同时阻止噪声或参数错误把速度积分到异常大。
     */
    max_electrical_speed = RPM2RADPS_f(fabsf(motor->ol_max_rpm) *
                                        fmaxf(motor->ol_pole_pairs, 1.0f) * 1.5f);
    s_sensorless.pll_speed_electrical =
        Motor_Sensorless_Clamp(s_sensorless.pll_speed_electrical,
                               -max_electrical_speed, max_electrical_speed);
}

void Motor_Sensorless_Init(void)
{
    /* 上电初始化：清空所有历史量，并明确告诉速度环“当前没有可信反馈”。 */
    memset(&s_sensorless, 0, sizeof(s_sensorless));
    s_sensorless.state = MOTOR_SENSORLESS_OPEN_LOOP;
    Motor_SpeedFeedback_Set(0.0f, false);
}

void Motor_Sensorless_Reset(void)
{
    /*
     * 停机、切换到 V/F、关闭无感或数值异常时调用。
     * 清零后必须重新经过 Seed、LOCKING 和接管流程，不能沿用旧角度。
     */
    memset(&s_sensorless, 0, sizeof(s_sensorless));
    s_sensorless.state = MOTOR_SENSORLESS_OPEN_LOOP;
    Motor_SpeedFeedback_Set(0.0f, false);
}

void Motor_Sensorless_SetAppliedVoltage(float v_alpha, float v_beta)
{
    /*
     * 该函数由 FOC 输出函数在完成反 Park 后调用。
     * 记录的是控制器本周期“命令给逆变器”的 α/β 电压，观测器会在下一
     * 周期与新采样电流配对使用。当前尚未补偿死区、管压降和母线纹波，
     * 因此它是实际电机端电压的近似值，而不是额外生成的测试电压。
     */
    s_sensorless.applied_v_alpha = v_alpha;
    s_sensorless.applied_v_beta = v_beta;
}

/*
 * 执行一次无感估算与状态机更新。
 *
 * 调用频率：与 I/F 电流环相同，每个 FOC 控制周期调用一次。
 * 输入数据：
 *   - motor->ia/ib/ic：本周期三相电流，单位 A；
 *   - applied_v_alpha/beta：上一周期保存的控制电压，单位 V；
 *   - dt：一次 FOC 周期的时间，单位 s；
 *   - R/L/flux：电阻 Ω、电感 H、永磁磁链 Wb。
 *
 * 执行顺序：
 *   1. 检查运行条件并整理电机参数；
 *   2. 三相电流做 Clarke 变换；
 *   3. 积分 Ortega 磁链模型；
 *   4. 从转子磁链计算原始角度；
 *   5. PLL 估算平滑角度和转速；
 *   6. 判断能否接管，或闭环后是否已经失锁；
 *   7. 输出渐变后的控制角度和速度有效标志。
 */
void Motor_Sensorless_Update(MOTOR_PARAM_S *motor, float dt)
{
    /* 电机模型参数，全部使用 SI 单位。 */
    float resistance;
    float inductance;
    float flux;
    float observer_gain;

    /* Clarke 变换后的定子电流。 */
    float i_alpha;
    float i_beta;

    /* 电感磁链 L×I，以及从定子磁链中分离出的转子磁链。 */
    float l_i_alpha;
    float l_i_beta;
    float rotor_flux_alpha;
    float rotor_flux_beta;

    /* Ortega 磁链幅值误差和定子磁链微分量。 */
    float magnitude_error;
    float x_alpha_dot;
    float x_beta_dot;

    /* 极对数用于电角速度/机械速度换算；speed_alpha 是 RPM 低通系数。 */
    float pole_pairs;
    float speed_alpha;
    float open_loop_rpm_abs;
    float observer_rpm_abs;
    float speed_error_limit;

    /* 四个锁定条件，必须全部为 true 才开始累计 lock_timer。 */
    bool flux_ok;
    bool speed_ok;
    bool direction_ok;
    bool phase_ok;
    bool lock_candidate;

    /*
     * 无感观测器只服务于正在运行的 I/F 电流闭环：
     *   - dt 非法时不能积分；
     *   - sensorless_enable=false 表示用户主动关闭；
     *   - V/F 模式没有可靠的电流闭环接管路径；
     *   - ol_enable=false 表示电机已停机。
     * 任一条件不满足都清空历史状态，防止下次启动沿用旧磁链和旧角度。
     */
    if (dt <= 0.0f || !motor->sensorless_enable ||
        motor->ol_mode != MOTOR_OL_MODE_IF || !motor->ol_enable) {
        Motor_Sensorless_Reset();
        return;
    }

    /*
     * 对关键模型参数做最后一道保护：R/L 不允许为负，额定磁链不能为 0。
     * 参数数量级必须正确，例如 400mΩ 要写成 0.4Ω，800uH 要写成 0.0008H。
     */
    resistance = fmaxf(motor->sensorless_motor_r, 0.0f);
    inductance = fmaxf(motor->sensorless_motor_l, 0.0f);
    flux = fmaxf(motor->sensorless_flux_linkage, 1.0e-6f);
    observer_gain = motor->sensorless_observer_gain;

    /*
     * 增益未配置时，根据磁链平方给一个可运行的回退值。
     * flux 越小，1/(flux²) 越大，所以错误的磁链单位会直接造成巨大增益。
     */
    if (observer_gain <= 0.0f) {
        observer_gain = 1000.0f / (flux * flux);
    }
    pole_pairs = fmaxf(motor->ol_pole_pairs, 1.0f);

    /* Ia/Ib/Ic -> 静止 α/β 坐标系。观测器不在旋转 D/Q 坐标系中计算。 */
    clarke_transform(motor->ia, motor->ib, motor->ic, &i_alpha, &i_beta);

    /* 第一次运行时先建立合理磁链初值，后续周期只做连续积分。 */
    if (!s_sensorless.initialized) {
        Motor_Sensorless_Seed(motor, i_alpha, i_beta);
    }

    /*
     * 先用“积分前”的定子磁链计算当前转子磁链：
     *
     *   ψrotor = ψstator - L×I
     *
     * rotor_flux_alpha/beta 应接近一个模长为 flux 的旋转矢量。
     */
    l_i_alpha = inductance * i_alpha;
    l_i_beta = inductance * i_beta;
    rotor_flux_alpha = s_sensorless.x_alpha - l_i_alpha;
    rotor_flux_beta = s_sensorless.x_beta - l_i_beta;

    /*
     * Ortega 幅值误差：
     *
     *   magnitude_error = flux² - |ψrotor|²
     *
     * 理想情况下转子永磁体磁链模长应接近额定 flux。当前实现使用单边
     * 修正：当估算磁链偏小时把正误差截为 0，不主动放大低速采样噪声；
     * 当估算磁链偏大时保留负误差，把磁链拉回额定磁链圆附近。
     */
    magnitude_error = flux * flux -
                      (rotor_flux_alpha * rotor_flux_alpha +
                       rotor_flux_beta * rotor_flux_beta);
    if (magnitude_error > 0.0f) {
        magnitude_error = 0.0f;
    }

    /*
     * 定子磁链微分方程：
     *
     *   dψstator/dt = V - R×I + Ortega 幅值修正项
     *
     * V-R×I 是电机电压方程；最后一项只用于抑制积分漂移和磁链发散。
     */
    x_alpha_dot = s_sensorless.applied_v_alpha - resistance * i_alpha +
                  0.5f * observer_gain * rotor_flux_alpha * magnitude_error;
    x_beta_dot = s_sensorless.applied_v_beta - resistance * i_beta +
                 0.5f * observer_gain * rotor_flux_beta * magnitude_error;
    /* 欧拉积分：新磁链 = 旧磁链 + 磁链变化率 × FOC 周期。 */
    s_sensorless.x_alpha += x_alpha_dot * dt;
    s_sensorless.x_beta += x_beta_dot * dt;

    /*
     * 参数错误、dt 错误或增益过大都可能使积分结果变成 NaN/Inf。
     * 一旦出现必须立即复位并声明速度无效，绝不能让非法角度进入 Park 变换，
     * 否则 D/Q 电流会被投影到错误方向并可能立即过流。
     */
    if (UTILS_IS_NAN(s_sensorless.x_alpha) || UTILS_IS_INF(s_sensorless.x_alpha) ||
        UTILS_IS_NAN(s_sensorless.x_beta) || UTILS_IS_INF(s_sensorless.x_beta)) {
        Motor_Sensorless_Reset();
        return;
    }

    /*
     * x_alpha/x_beta 刚刚完成积分，因此重新计算一次最新转子磁链。
     * 模长用于判断观测是否可信，方向 atan2(β, α) 就是转子电角度。
     */
    rotor_flux_alpha = s_sensorless.x_alpha - inductance * i_alpha;
    rotor_flux_beta = s_sensorless.x_beta - inductance * i_beta;
    s_sensorless.flux_magnitude = NORM2_f(rotor_flux_alpha, rotor_flux_beta);
    s_sensorless.observer_angle =
        utils_fast_atan2(rotor_flux_beta, rotor_flux_alpha) +
        motor->sensorless_phase_offset;

    /* phase_offset 用于补偿固定相位偏差；归一化后角度保持在标准周期内。 */
    utils_norm_angle_rad(&s_sensorless.observer_angle);

    /*
     * 原始 atan2 角度先进入 PLL，不能直接拿去控制电流环。
     * PLL 输出电角速度；除以极对数后才是机械角速度，再换算成 RPM。
     */
    Motor_Sensorless_RunPll(motor, dt);
    s_sensorless.mechanical_rpm =
        RADPS2RPM_f(s_sensorless.pll_speed_electrical) / pole_pairs;
    /*
     * 对机械 RPM 再做一次一阶低通：
     *   filtered += alpha × (raw - filtered)
     * alpha 越接近 1 响应越快但噪声越大，越接近 0 越平滑但延迟越大。
     */
    speed_alpha = Motor_Sensorless_Clamp(motor->sensorless_speed_filter,
                                         0.0f, 1.0f);
    s_sensorless.filtered_rpm += speed_alpha *
                                 (s_sensorless.mechanical_rpm -
                                  s_sensorless.filtered_rpm);

    /*
     * phase_error 比较“PLL 观测角”和“I/F 强制角”。使用最短角度差函数，
     * 可以正确处理 +π/-π 交界，避免 179° 与 -179° 被误判为相差 358°。
     */
    s_sensorless.phase_error =
        utils_angle_difference_rad(s_sensorless.pll_angle, motor->ol_theta);

    /* 条件 1：磁链模长应在额定值的 50%~150%，过小或过大都不可信。 */
    flux_ok = s_sensorless.flux_magnitude > flux * 0.5f &&
              s_sensorless.flux_magnitude < flux * 1.5f;

    /*
     * 条件 2：不仅要超过接管门槛，还必须与 I/F 当前斜坡速度大致一致。
     * 只判断“观测值大于 500RPM”可能把噪声或错误极对数当成有效转速。
     * 这里允许 25% 或至少 100RPM 的偏差，兼顾启动负载角和低速抖动；
     * 偏差再大说明转子很可能已经明显落后，不应冒险切入观测角。
     */
    open_loop_rpm_abs = fabsf(motor->ol_current_rpm);
    observer_rpm_abs = fabsf(s_sensorless.filtered_rpm);
    speed_error_limit = fmaxf(open_loop_rpm_abs * 0.25f, 100.0f);
    speed_ok = open_loop_rpm_abs >= motor->sensorless_transition_rpm &&
               observer_rpm_abs >= motor->sensorless_transition_rpm &&
               fabsf(observer_rpm_abs - open_loop_rpm_abs) <= speed_error_limit;

    /* 条件 3：观测转速方向必须和 I/F 当前方向一致。零速时暂不检查方向。 */
    direction_ok = motor->ol_current_rpm == 0.0f ||
                   SIGN(s_sensorless.filtered_rpm) == SIGN(motor->ol_current_rpm);

    /* 条件 4：观测角与强制角足够接近，避免接管时 Park 角度突然跳变。 */
    phase_ok = fabsf(s_sensorless.phase_error) <=
               motor->sensorless_phase_tolerance;

    /* 保存每项判断结果，Shell 和首发故障快照可以直接指出阻塞原因。 */
    s_sensorless.flux_ok = flux_ok;
    s_sensorless.speed_ok = speed_ok;
    s_sensorless.direction_ok = direction_ok;
    s_sensorless.phase_ok = phase_ok;

    /* 四项同时成立才是一次有效锁定候选，但仍需连续保持 lock_time。 */
    lock_candidate = flux_ok && speed_ok && direction_ok && phase_ok;

    /*
     * 尚未闭环时有两个可见状态：
     *   OPEN_LOOP：至少一个锁定条件不满足，继续完全使用 I/F 角度；
     *   LOCKING：四个条件暂时满足，正在累计连续稳定时间。
     *
     * 任一条件中途失败，lock_timer 立即清零，必须重新连续稳定一整段时间。
     */
    if (s_sensorless.state != MOTOR_SENSORLESS_CLOSED) {
        s_sensorless.state = lock_candidate ? MOTOR_SENSORLESS_LOCKING :
                                              MOTOR_SENSORLESS_OPEN_LOOP;
        s_sensorless.lock_timer = lock_candidate ?
            s_sensorless.lock_timer + dt : 0.0f;

        /*
         * sensorless_takeover 是最后一道人工授权开关。即使状态已经 LOCKING，
         * 默认 takeover=false 时也只观察数据，不会让观测角控制电机。
         */
        if (motor->sensorless_takeover &&
            s_sensorless.lock_timer >= motor->sensorless_lock_time) {
            /*
             * 速度 PI 接管 Iq 前，用当前 I/F 启动电流预置积分项。
             * 如果直接从 0A 开始，接管瞬间 Iq 会突然下降，电机容易减速失锁。
             */
            float startup_iq = SIGN(motor->ol_current_rpm) * fabsf(motor->ol_iq_set);

            /* 切到 CLOSED 后 blend 从 0 开始，角度不会在这一拍直接跳到 PLL。 */
            s_sensorless.state = MOTOR_SENSORLESS_CLOSED;
            s_sensorless.blend = 0.0f;
            s_sensorless.loss_timer = 0.0f;
            Motor_SpeedLoop_PrimeIq(startup_iq);
        }
    } else {
        /*
         * 已闭环后持续检查三个严重失锁条件：磁链异常、方向相反、转速过低。
         * 相位误差不再单独作为失锁条件，因为 CLOSED 状态会持续把 ol_theta
         * 同步到控制角；此时它不再代表一条独立的开环参考轨迹。
         */
        bool wrong_direction = motor->ol_current_rpm != 0.0f &&
                               SIGN(s_sensorless.filtered_rpm) !=
                               SIGN(motor->ol_current_rpm);
        bool lost = !flux_ok || wrong_direction ||
                    fabsf(s_sensorless.filtered_rpm) < motor->sensorless_fallback_rpm;

        /* 短暂噪声不会立刻回退，只有失锁条件连续超过 loss_time 才处理。 */
        s_sensorless.loss_timer = lost ? s_sensorless.loss_timer + dt : 0.0f;
        if (s_sensorless.loss_timer >= motor->sensorless_loss_time) {
            bool above_safe_open_loop_speed =
                motor->sensorless_openloop_max_rpm > 0.0f &&
                fabsf(motor->ol_current_rpm) >
                motor->sensorless_openloop_max_rpm;

            /*
             * 低速失锁时允许回退到 I/F，因为当前强制转速仍在启动安全区内。
             * 但高速时绝不能回退：旧逻辑会让 I/F 从当前高速慢慢降到上限，
             * 这段时间转子可能已跟不上强制角，正是“跑一会突然过流停机”的
             * 另一条路径。因此高于 I/F 安全上限时先锁存失速故障，下一次
             * 保护入口（最迟一个 ADC 周期）会关闭输出。
             */
            if (above_safe_open_loop_speed) {
                Motor_Protect_TriggerFault(FAULT_CODE_MOTOR_SPEED_LOSS, motor);
            }

            /*
             * 无论低速回退还是等待故障停机，都撤销观测角和速度有效标志。
             * ol_theta 在 CLOSED 时一直同步到 control_angle，因此切回 I/F
             * 不会跳回接管前的旧角度。
             */
            s_sensorless.state = MOTOR_SENSORLESS_OPEN_LOOP;
            s_sensorless.lock_timer = 0.0f;
            s_sensorless.loss_timer = 0.0f;
            s_sensorless.blend = 0.0f;
            Motor_SpeedFeedback_Set(0.0f, false);
        }
    }

    /*
     * CLOSED 状态下把控制角从 I/F 平滑过渡到 PLL：
     *
     *   blend += dt / blend_time
     *   control = open_loop + shortest(pll - open_loop) × blend
     *
     * blend=0 时完全使用 I/F 角度，blend=1 时完全使用 PLL 角度。
     */
    if (s_sensorless.state == MOTOR_SENSORLESS_CLOSED) {
        float blend_step;
        float angle_difference;

        /* blend_time<=0 表示用户要求一拍完成切换，否则按设定时间线性增加。 */
        blend_step = motor->sensorless_blend_time > 0.0f ?
                     dt / motor->sensorless_blend_time : 1.0f;
        s_sensorless.blend = Motor_Sensorless_Clamp(s_sensorless.blend + blend_step,
                                                    0.0f, 1.0f);
        /* 始终沿最短方向靠近 PLL 角度，避免跨 ±π 时绕远路。 */
        angle_difference = utils_angle_difference_rad(s_sensorless.pll_angle,
                                                       motor->ol_theta);
        s_sensorless.control_angle = motor->ol_theta +
                                     angle_difference * s_sensorless.blend;
        utils_norm_angle_rad(&s_sensorless.control_angle);

        /*
         * 同步 I/F 角度有两个作用：
         *   1. 下一周期的渐变从当前实际控制角继续，不会重复从旧角度开始；
         *   2. 若发生失锁回退，I/F 会从当前角度接着积分，不产生角度阶跃。
         */
        motor->ol_theta = s_sensorless.control_angle;

        /* 只有 CLOSED 才发布有效速度，速度 PI、堵转和失速保护才会使用它。 */
        Motor_SpeedFeedback_Set(s_sensorless.filtered_rpm, true);
    } else {
        /* OPEN_LOOP/LOCKING 仍使用 I/F 角度，并明确禁止速度闭环使用估算 RPM。 */
        s_sensorless.control_angle = motor->ol_theta;
        Motor_SpeedFeedback_Set(0.0f, false);
    }
}

float Motor_Sensorless_GetControlAngle(float open_loop_angle)
{
    /*
     * 电流环每周期调用此函数选择 Park 角度。
     * 非 CLOSED 状态必须原样返回调用者提供的 I/F 角度；只有完成锁定并
     * 获得接管授权后，才返回已经做过渐变处理的 control_angle。
     */
    return s_sensorless.state == MOTOR_SENSORLESS_CLOSED ?
           s_sensorless.control_angle : open_loop_angle;
}

motor_sensorless_state_t Motor_Sensorless_GetState(void)
{
    /* 供 speed Shell 命令显示 OPEN_LOOP、LOCKING 或 CLOSED。 */
    return s_sensorless.state;
}

float Motor_Sensorless_GetObserverAngle(void)
{
    /*
     * 对外返回 PLL 平滑角，而不是 observer_angle 原始 atan2 角。
     * 调试时看到的是控制系统实际准备使用的观测电角度，单位 rad。
     */
    return s_sensorless.pll_angle;
}

float Motor_Sensorless_GetFluxMagnitude(void)
{
    /* 返回转子磁链估算模长，单位 Wb；应接近 sensorless_flux_linkage。 */
    return s_sensorless.flux_magnitude;
}

float Motor_Sensorless_GetMechanicalRpm(void)
{
    /* 返回低通后的机械转速 RPM，正负号表示正反转。 */
    return s_sensorless.filtered_rpm;
}

float Motor_Sensorless_GetPhaseError(void)
{
    /* 返回 PLL 观测角与 I/F 强制角的最短角度差，单位 rad。 */
    return s_sensorless.phase_error;
}

void Motor_Sensorless_GetDiagnostics(motor_sensorless_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->flux_ok = s_sensorless.flux_ok;
    diagnostics->speed_ok = s_sensorless.speed_ok;
    diagnostics->direction_ok = s_sensorless.direction_ok;
    diagnostics->phase_ok = s_sensorless.phase_ok;
    diagnostics->lock_timer = s_sensorless.lock_timer;
    diagnostics->blend = s_sensorless.blend;
    diagnostics->applied_voltage =
        NORM2_f(s_sensorless.applied_v_alpha, s_sensorless.applied_v_beta);
}

bool Motor_Sensorless_IsClosedLoop(void)
{
    /* true 表示观测角已经获准参与 Park 变换，不能只理解为“观测器已开启”。 */
    return s_sensorless.state == MOTOR_SENSORLESS_CLOSED;
}

bool Motor_Sensorless_IsTakeoverComplete(void)
{
    /* blend 经过 Clamp 后会精确到 1.0，达到该值才算完全使用观测角。 */
    return s_sensorless.state == MOTOR_SENSORLESS_CLOSED &&
           s_sensorless.blend >= 1.0f;
}

#ifdef __cplusplus
}
#endif
