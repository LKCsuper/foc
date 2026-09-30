/*
 * @Description: 无感观测器：Ortega 磁链观测 + PLL 锁相与角度接管（支持状态实例）
 * @Author: lkc
 *
 * 学习要点见 motor_sensorless.h。核心 UpdateState 经 effects 报告副作用；
 * 默认包装 ApplyEffects 再接到速度环/保护，便于主机双实例测试。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motor_param.h"
#include "motor_sensorless.h"
#include "motor_foc.h"
#include "motor_math.h"
#include "motor_speed.h"
#include "motor_protect.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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

/* 默认单实例包装，步骤 2 过渡期保留；学习/测试优先用 *State API。 */
static motor_sensorless_runtime_t s_sensorless;

/**
 * @description: 将 UpdateState 产生的副作用落到速度环/保护（仅默认包装使用）。
 * @param {MOTOR_PARAM_S *} motor 电机对象，可为 NULL（仅触发保护需要）
 * @param {const motor_sensorless_effects_t *} effects 观测器副作用输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Sensorless_ApplyEffects(MOTOR_PARAM_S *motor, const motor_sensorless_effects_t *effects)
{
    if (effects == NULL) {
        return;
    }
    if (effects->request_speed_loss_fault && motor != NULL) {
        Motor_Protect_TriggerFault(FAULT_CODE_MOTOR_SPEED_LOSS, motor);
    }
    if (effects->prime_speed_iq) {
        Motor_SpeedLoop_PrimeIq(effects->prime_iq);
    }
    Motor_SpeedFeedback_Set(effects->feedback_rpm, effects->feedback_valid);
}

/**
 * @description: 把观测器本拍要上报的副作用清零。
 * @param {motor_sensorless_effects_t *} effects  副作用输出，为空则直接返回
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Sensorless_ClearEffects(motor_sensorless_effects_t *effects)
{
    if (effects == NULL) {
        return;
    }
    memset(effects, 0, sizeof(*effects));
}

/**
 * @description: 把数值限制到给定闭区间。
 * @param {float} value    输入值
 * @param {float} minimum  下限
 * @param {float} maximum  上限
 * @return {float} 限制后的值
 * @author: lkc
 */
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
/**
 * @description: 用当前 I/F 强制角初始化磁链积分器。
 * @detail: 按 L×I 加假定永磁磁链给出初值，避免第一次 atan2 从零磁链出发。这不是真实转子角。
 * @param {motor_sensorless_runtime_t *} rt  观测器运行时
 * @param {const MOTOR_PARAM_S *} motor     电感、磁链和开环角
 * @param {float} i_alpha                   Iα (A)
 * @param {float} i_beta                    Iβ (A)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Sensorless_Seed(motor_sensorless_runtime_t *rt, const MOTOR_PARAM_S *motor, float i_alpha,
                                  float i_beta)
{
    float inductance = motor->sensorless_motor_l;
    float flux = motor->sensorless_flux_linkage;

    /* L×I 是定子电感磁链，flux×cos/sin 是假定的永磁体磁链。 */
    rt->x_alpha = inductance * i_alpha + flux * cosf(motor->ol_theta);
    rt->x_beta = inductance * i_beta + flux * sinf(motor->ol_theta);

    /* 原始角、PLL 角和控制角从同一角度出发，避免启动瞬间发生角度阶跃。 */
    rt->observer_angle = motor->ol_theta;
    rt->pll_angle = motor->ol_theta;
    rt->pll_speed_electrical = 0.0f;
    rt->control_angle = motor->ol_theta;
    rt->initialized = true;
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
/**
 * @description: 用二阶 PLL 跟踪观测电角并积分出电角速度。
 * @detail: 速度限制在最高机械转速对应电角速度的 1.5 倍以内。单位是 rad/s。
 * @param {motor_sensorless_runtime_t *} rt  观测器运行时
 * @param {const MOTOR_PARAM_S *} motor     PLL 增益和转速上限
 * @param {float} dt                        步长 (s)
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Sensorless_RunPll(motor_sensorless_runtime_t *rt, const MOTOR_PARAM_S *motor, float dt)
{
    float delta = utils_angle_difference_rad(rt->observer_angle, rt->pll_angle);
    float max_electrical_speed;

    /* 先用“当前速度 + 比例相位校正”推进 PLL 角度。 */
    rt->pll_angle += (rt->pll_speed_electrical + motor->sensorless_pll_kp * delta) * dt;
    utils_norm_angle_rad(&rt->pll_angle);

    /* 再积分相位误差，得到能够长期跟踪转速的电角速度。 */
    rt->pll_speed_electrical += motor->sensorless_pll_ki * delta * dt;

    /*
     * 将 PLL 速度限制在“最高机械转速 × 极对数 × 1.5”以内。
     * 1.5 倍留出加速余量，同时阻止噪声或参数错误把速度积分到异常大。
     */
    max_electrical_speed = RPM2RADPS_f(fabsf(motor->ol_max_rpm) * fmaxf(motor->ol_pole_pairs, 1.0f) * 1.5f);
    rt->pll_speed_electrical =
        Motor_Sensorless_Clamp(rt->pll_speed_electrical, -max_electrical_speed, max_electrical_speed);
}

/**
 * @description: 初始化指定观测器运行时状态。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_InitState(motor_sensorless_runtime_t *rt)
{
    if (rt == NULL) {
        return;
    }
    memset(rt, 0, sizeof(*rt));
    rt->state = MOTOR_SENSORLESS_OPEN_LOOP;
}

/**
 * @description: 复位指定观测器运行时状态。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_ResetState(motor_sensorless_runtime_t *rt)
{
    /*
     * 停机、切换到 V/F、关闭无感或数值异常时调用。
     * 清零后必须重新经过 Seed、LOCKING 和接管流程，不能沿用旧角度。
     */
    Motor_Sensorless_InitState(rt);
}

/**
 * @description: 记录指定实例本周期实际下发的 α/β 电压。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象
 * @param {float} v_alpha  α 轴电压 (V)
 * @param {float} v_beta   β 轴电压 (V)
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_SetAppliedVoltageState(motor_sensorless_runtime_t *rt, float v_alpha, float v_beta)
{
    /*
     * 该函数由 FOC 输出函数在完成反 Park 后调用。
     * 记录的是控制器本周期“命令给逆变器”的 α/β 电压，观测器会在下一
     * 周期与新采样电流配对使用。当前尚未补偿死区、管压降和母线纹波，
     * 因此它是实际电机端电压的近似值，而不是额外生成的测试电压。
     */
    if (rt == NULL) {
        return;
    }
    rt->applied_v_alpha = v_alpha;
    rt->applied_v_beta = v_beta;
}

/**
 * @description: 推进指定实例的 Ortega/PLL/接管状态机一拍。
 * @detail: Clarke → 磁链积分 → atan2 → PLL → 锁定/失锁 → 角度渐变；
 *          副作用写入 effects，不直接调速度环/保护。
 * @param {motor_sensorless_runtime_t *} rt  运行时对象
 * @param {MOTOR_PARAM_S *} motor  电机参数与采样
 * @param {float} dt  FOC 控制周期 (s)
 * @param {motor_sensorless_effects_t *} effects  副作用输出，可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_UpdateState(motor_sensorless_runtime_t *rt, MOTOR_PARAM_S *motor, float dt,
                                  motor_sensorless_effects_t *effects)
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
    if (rt == NULL || motor == NULL) {
        return;
    }
    Motor_Sensorless_ClearEffects(effects);

    if (dt <= 0.0f || !motor->sensorless_enable || motor->ol_mode != MOTOR_OL_MODE_IF || !motor->ol_enable) {
        Motor_Sensorless_ResetState(rt);
        if (effects != NULL) {
            effects->feedback_valid = false;
        }
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
    if (!rt->initialized) {
        Motor_Sensorless_Seed(rt, motor, i_alpha, i_beta);
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
    rotor_flux_alpha = rt->x_alpha - l_i_alpha;
    rotor_flux_beta = rt->x_beta - l_i_beta;

    /*
     * Ortega 幅值误差：
     *
     *   magnitude_error = flux² - |ψrotor|²
     *
     * 理想情况下转子永磁体磁链模长应接近额定 flux。当前实现使用单边
     * 修正：当估算磁链偏小时把正误差截为 0，不主动放大低速采样噪声；
     * 当估算磁链偏大时保留负误差，把磁链拉回额定磁链圆附近。
     */
    magnitude_error = flux * flux - (rotor_flux_alpha * rotor_flux_alpha + rotor_flux_beta * rotor_flux_beta);
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
    x_alpha_dot =
        rt->applied_v_alpha - resistance * i_alpha + 0.5f * observer_gain * rotor_flux_alpha * magnitude_error;
    x_beta_dot = rt->applied_v_beta - resistance * i_beta + 0.5f * observer_gain * rotor_flux_beta * magnitude_error;
    /* 欧拉积分：新磁链 = 旧磁链 + 磁链变化率 × FOC 周期。 */
    rt->x_alpha += x_alpha_dot * dt;
    rt->x_beta += x_beta_dot * dt;

    /*
     * 参数错误、dt 错误或增益过大都可能使积分结果变成 NaN/Inf。
     * 一旦出现必须立即复位并声明速度无效，绝不能让非法角度进入 Park 变换，
     * 否则 D/Q 电流会被投影到错误方向并可能立即过流。
     */
    if (UTILS_IS_NAN(rt->x_alpha) || UTILS_IS_INF(rt->x_alpha) || UTILS_IS_NAN(rt->x_beta) ||
        UTILS_IS_INF(rt->x_beta)) {
        Motor_Sensorless_ResetState(rt);
        if (effects != NULL) {
            effects->feedback_valid = false;
        }
        return;
    }

    /*
     * x_alpha/x_beta 刚刚完成积分，因此重新计算一次最新转子磁链。
     * 模长用于判断观测是否可信，方向 atan2(β, α) 就是转子电角度。
     */
    rotor_flux_alpha = rt->x_alpha - inductance * i_alpha;
    rotor_flux_beta = rt->x_beta - inductance * i_beta;
    rt->flux_magnitude = NORM2_f(rotor_flux_alpha, rotor_flux_beta);
    rt->observer_angle = utils_fast_atan2(rotor_flux_beta, rotor_flux_alpha) + motor->sensorless_phase_offset;

    /* phase_offset 用于补偿固定相位偏差；归一化后角度保持在标准周期内。 */
    utils_norm_angle_rad(&rt->observer_angle);

    /*
     * 原始 atan2 角度先进入 PLL，不能直接拿去控制电流环。
     * PLL 输出电角速度；除以极对数后才是机械角速度，再换算成 RPM。
     */
    Motor_Sensorless_RunPll(rt, motor, dt);
    rt->mechanical_rpm = RADPS2RPM_f(rt->pll_speed_electrical) / pole_pairs;
    /*
     * 对机械 RPM 再做一次一阶低通：
     *   filtered += alpha × (raw - filtered)
     * alpha 越接近 1 响应越快但噪声越大，越接近 0 越平滑但延迟越大。
     */
    speed_alpha = Motor_Sensorless_Clamp(motor->sensorless_speed_filter, 0.0f, 1.0f);
    rt->filtered_rpm += speed_alpha * (rt->mechanical_rpm - rt->filtered_rpm);

    /*
     * phase_error 比较“PLL 观测角”和“I/F 强制角”。使用最短角度差函数，
     * 可以正确处理 +π/-π 交界，避免 179° 与 -179° 被误判为相差 358°。
     */
    rt->phase_error = utils_angle_difference_rad(rt->pll_angle, motor->ol_theta);

    /* 条件 1：磁链模长应在额定值的 50%~150%，过小或过大都不可信。 */
    flux_ok = rt->flux_magnitude > flux * 0.5f && rt->flux_magnitude < flux * 1.5f;

    /*
     * 条件 2：不仅要超过接管门槛，还必须与 I/F 当前斜坡速度大致一致。
     * 只判断“观测值大于 500RPM”可能把噪声或错误极对数当成有效转速。
     * 这里允许 25% 或至少 100RPM 的偏差，兼顾启动负载角和低速抖动；
     * 偏差再大说明转子很可能已经明显落后，不应冒险切入观测角。
     */
    open_loop_rpm_abs = fabsf(motor->ol_current_rpm);
    observer_rpm_abs = fabsf(rt->filtered_rpm);
    speed_error_limit = fmaxf(open_loop_rpm_abs * 0.25f, 100.0f);
    speed_ok = open_loop_rpm_abs >= motor->sensorless_transition_rpm &&
               observer_rpm_abs >= motor->sensorless_transition_rpm &&
               fabsf(observer_rpm_abs - open_loop_rpm_abs) <= speed_error_limit;

    /* 条件 3：观测转速方向必须和 I/F 当前方向一致。零速时暂不检查方向。 */
    direction_ok = motor->ol_current_rpm == 0.0f || SIGN(rt->filtered_rpm) == SIGN(motor->ol_current_rpm);

    /* 条件 4：观测角与强制角足够接近，避免接管时 Park 角度突然跳变。 */
    phase_ok = fabsf(rt->phase_error) <= motor->sensorless_phase_tolerance;

    /* 保存每项判断结果，Shell 和首发故障快照可以直接指出阻塞原因。 */
    rt->flux_ok = flux_ok;
    rt->speed_ok = speed_ok;
    rt->direction_ok = direction_ok;
    rt->phase_ok = phase_ok;

    /* 四项同时成立才是一次有效锁定候选，但仍需连续保持 lock_time。 */
    lock_candidate = flux_ok && speed_ok && direction_ok && phase_ok;

    /*
     * 尚未闭环时有两个可见状态：
     *   OPEN_LOOP：至少一个锁定条件不满足，继续完全使用 I/F 角度；
     *   LOCKING：四个条件暂时满足，正在累计连续稳定时间。
     *
     * 任一条件中途失败，lock_timer 立即清零，必须重新连续稳定一整段时间。
     */
    if (rt->state != MOTOR_SENSORLESS_CLOSED) {
        rt->state = lock_candidate ? MOTOR_SENSORLESS_LOCKING : MOTOR_SENSORLESS_OPEN_LOOP;
        rt->lock_timer = lock_candidate ? rt->lock_timer + dt : 0.0f;

        /*
         * sensorless_takeover 是最后一道人工授权开关。即使状态已经 LOCKING，
         * 默认 takeover=false 时也只观察数据，不会让观测角控制电机。
         */
        if (motor->sensorless_takeover && rt->lock_timer >= motor->sensorless_lock_time) {
            /*
             * 速度 PI 接管 Iq 前，用当前 I/F 启动电流预置积分项。
             * 如果直接从 0A 开始，接管瞬间 Iq 会突然下降，电机容易减速失锁。
             */
            float startup_iq = SIGN(motor->ol_current_rpm) * fabsf(motor->ol_iq_set);

            /* 切到 CLOSED 后 blend 从 0 开始，角度不会在这一拍直接跳到 PLL。 */
            rt->state = MOTOR_SENSORLESS_CLOSED;
            rt->blend = 0.0f;
            rt->loss_timer = 0.0f;
            if (effects != NULL) {
                effects->prime_speed_iq = true;
                effects->prime_iq = startup_iq;
            }
        }
    } else {
        /*
         * 已闭环后持续检查三个严重失锁条件：磁链异常、方向相反、转速过低。
         * 相位误差不再单独作为失锁条件，因为 CLOSED 状态会持续把 ol_theta
         * 同步到控制角；此时它不再代表一条独立的开环参考轨迹。
         */
        bool wrong_direction = motor->ol_current_rpm != 0.0f && SIGN(rt->filtered_rpm) != SIGN(motor->ol_current_rpm);
        bool lost = !flux_ok || wrong_direction || fabsf(rt->filtered_rpm) < motor->sensorless_fallback_rpm;

        /* 短暂噪声不会立刻回退，只有失锁条件连续超过 loss_time 才处理。 */
        rt->loss_timer = lost ? rt->loss_timer + dt : 0.0f;
        if (rt->loss_timer >= motor->sensorless_loss_time) {
            bool above_safe_open_loop_speed = motor->sensorless_openloop_max_rpm > 0.0f &&
                                              fabsf(motor->ol_current_rpm) > motor->sensorless_openloop_max_rpm;

            /*
             * 低速失锁时允许回退到 I/F，因为当前强制转速仍在启动安全区内。
             * 但高速时绝不能回退：旧逻辑会让 I/F 从当前高速慢慢降到上限，
             * 这段时间转子可能已跟不上强制角，正是“跑一会突然过流停机”的
             * 另一条路径。因此高于 I/F 安全上限时先锁存失速故障，下一次
             * 保护入口（最迟一个 ADC 周期）会关闭输出。
             */
            if (above_safe_open_loop_speed && effects != NULL) {
                effects->request_speed_loss_fault = true;
            }

            /*
             * 无论低速回退还是等待故障停机，都撤销观测角和速度有效标志。
             * ol_theta 在 CLOSED 时一直同步到 control_angle，因此切回 I/F
             * 不会跳回接管前的旧角度。
             */
            rt->state = MOTOR_SENSORLESS_OPEN_LOOP;
            rt->lock_timer = 0.0f;
            rt->loss_timer = 0.0f;
            rt->blend = 0.0f;
            if (effects != NULL) {
                effects->feedback_valid = false;
                effects->feedback_rpm = 0.0f;
            }
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
    if (rt->state == MOTOR_SENSORLESS_CLOSED) {
        float blend_step;
        float angle_difference;

        /* blend_time<=0 表示用户要求一拍完成切换，否则按设定时间线性增加。 */
        blend_step = motor->sensorless_blend_time > 0.0f ? dt / motor->sensorless_blend_time : 1.0f;
        rt->blend = Motor_Sensorless_Clamp(rt->blend + blend_step, 0.0f, 1.0f);
        /* 始终沿最短方向靠近 PLL 角度，避免跨 ±π 时绕远路。 */
        angle_difference = utils_angle_difference_rad(rt->pll_angle, motor->ol_theta);
        rt->control_angle = motor->ol_theta + angle_difference * rt->blend;
        utils_norm_angle_rad(&rt->control_angle);

        /*
         * 同步 I/F 角度有两个作用：
         *   1. 下一周期的渐变从当前实际控制角继续，不会重复从旧角度开始；
         *   2. 若发生失锁回退，I/F 会从当前角度接着积分，不产生角度阶跃。
         */
        motor->ol_theta = rt->control_angle;

        /* 只有 CLOSED 才发布有效速度，速度 PI、堵转和失速保护才会使用它。 */
        if (effects != NULL) {
            effects->feedback_valid = true;
            effects->feedback_rpm = rt->filtered_rpm;
        }
    } else {
        /* OPEN_LOOP/LOCKING 仍使用 I/F 角度，并明确禁止速度闭环使用估算 RPM。 */
        rt->control_angle = motor->ol_theta;
        if (effects != NULL) {
            effects->feedback_valid = false;
            effects->feedback_rpm = 0.0f;
        }
    }
}

/**
 * @description: 获取指定实例本周期 Park 应用电角度。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @param {float} open_loop_angle  I/F 强制电角度 (rad)
 * @return {float} 控制用电角度 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetControlAngleState(const motor_sensorless_runtime_t *rt, float open_loop_angle)
{
    if (rt == NULL || rt->state != MOTOR_SENSORLESS_CLOSED) {
        return open_loop_angle;
    }
    return rt->control_angle;
}

/**
 * @description: 读取指定实例状态机。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {motor_sensorless_state_t} OPEN_LOOP / LOCKING / CLOSED
 * @author: lkc
 */
motor_sensorless_state_t Motor_Sensorless_GetStateState(const motor_sensorless_runtime_t *rt)
{
    return rt == NULL ? MOTOR_SENSORLESS_OPEN_LOOP : rt->state;
}

/**
 * @description: 读取指定实例 PLL 平滑观测电角。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 观测电角度 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetObserverAngleState(const motor_sensorless_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->pll_angle;
}

/**
 * @description: 读取指定实例转子磁链模长。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 磁链模长 (Wb)
 * @author: lkc
 */
float Motor_Sensorless_GetFluxMagnitudeState(const motor_sensorless_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->flux_magnitude;
}

/**
 * @description: 读取指定实例低通机械转速。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_Sensorless_GetMechanicalRpmState(const motor_sensorless_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->filtered_rpm;
}

/**
 * @description: 读取指定实例相位误差。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {float} 相位误差 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetPhaseErrorState(const motor_sensorless_runtime_t *rt)
{
    return rt == NULL ? 0.0f : rt->phase_error;
}

/**
 * @description: 拷贝指定实例接管诊断。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @param {motor_sensorless_diagnostics_t *} diagnostics  输出
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_GetDiagnosticsState(const motor_sensorless_runtime_t *rt,
                                          motor_sensorless_diagnostics_t *diagnostics)
{
    if (rt == NULL || diagnostics == NULL) {
        return;
    }
    diagnostics->flux_ok = rt->flux_ok;
    diagnostics->speed_ok = rt->speed_ok;
    diagnostics->direction_ok = rt->direction_ok;
    diagnostics->phase_ok = rt->phase_ok;
    diagnostics->lock_timer = rt->lock_timer;
    diagnostics->blend = rt->blend;
    diagnostics->applied_voltage = NORM2_f(rt->applied_v_alpha, rt->applied_v_beta);
}

/**
 * @description: 查询指定实例是否已 CLOSED。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {bool} true=观测角参与 Park
 * @author: lkc
 */
bool Motor_Sensorless_IsClosedLoopState(const motor_sensorless_runtime_t *rt)
{
    return rt != NULL && rt->state == MOTOR_SENSORLESS_CLOSED;
}

/**
 * @description: 查询指定实例角度渐变是否完成。
 * @param {const motor_sensorless_runtime_t *} rt  运行时对象
 * @return {bool} true=CLOSED 且 blend≥1
 * @author: lkc
 */
bool Motor_Sensorless_IsTakeoverCompleteState(const motor_sensorless_runtime_t *rt)
{
    return rt != NULL && rt->state == MOTOR_SENSORLESS_CLOSED && rt->blend >= 1.0f;
}

/**
 * @description: 上电初始化默认单实例观测器，并声明速度反馈无效。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Init(void)
{
    Motor_Sensorless_InitState(&s_sensorless);
    Motor_SpeedFeedback_Set(0.0f, false);
}

/**
 * @description: 复位默认实例观测器，并声明速度反馈无效。
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Reset(void)
{
    Motor_Sensorless_ResetState(&s_sensorless);
    Motor_SpeedFeedback_Set(0.0f, false);
}

/**
 * @description: 记录默认实例本周期实际下发的 α/β 电压。
 * @param {float} v_alpha  α 轴电压 (V)
 * @param {float} v_beta   β 轴电压 (V)
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_SetAppliedVoltage(float v_alpha, float v_beta)
{
    Motor_Sensorless_SetAppliedVoltageState(&s_sensorless, v_alpha, v_beta);
}

/**
 * @description: 默认实例更新观测器，并落地速度环/保护副作用。
 * @param {MOTOR_PARAM_S *} motor  电机实时参数
 * @param {float} dt  控制周期 (s)
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_Update(MOTOR_PARAM_S *motor, float dt)
{
    motor_sensorless_effects_t effects;

    Motor_Sensorless_UpdateState(&s_sensorless, motor, dt, &effects);
    Motor_Sensorless_ApplyEffects(motor, &effects);
}

/**
 * @description: 获取默认实例本周期 Park 变换应使用的电角度。
 * @param {float} open_loop_angle  I/F 强制电角度 (rad)
 * @return {float} 控制用电角度 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetControlAngle(float open_loop_angle)
{
    return Motor_Sensorless_GetControlAngleState(&s_sensorless, open_loop_angle);
}

/**
 * @description: 获取默认实例观测器状态机当前状态。
 * @return {motor_sensorless_state_t} OPEN_LOOP、LOCKING 或 CLOSED
 * @author: lkc
 */
motor_sensorless_state_t Motor_Sensorless_GetState(void)
{
    return Motor_Sensorless_GetStateState(&s_sensorless);
}

/**
 * @description: 获取默认实例 PLL 平滑后的观测电角度。
 * @return {float} 观测电角度 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetObserverAngle(void)
{
    return Motor_Sensorless_GetObserverAngleState(&s_sensorless);
}

/**
 * @description: 获取默认实例估算转子磁链模长。
 * @return {float} 磁链模长 (Wb)
 * @author: lkc
 */
float Motor_Sensorless_GetFluxMagnitude(void)
{
    return Motor_Sensorless_GetFluxMagnitudeState(&s_sensorless);
}

/**
 * @description: 获取默认实例低通后的机械转速。
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_Sensorless_GetMechanicalRpm(void)
{
    return Motor_Sensorless_GetMechanicalRpmState(&s_sensorless);
}

/**
 * @description: 获取默认实例相位误差。
 * @return {float} 相位误差 (rad)
 * @author: lkc
 */
float Motor_Sensorless_GetPhaseError(void)
{
    return Motor_Sensorless_GetPhaseErrorState(&s_sensorless);
}

/**
 * @description: 拷贝默认实例接管诊断数据。
 * @param {motor_sensorless_diagnostics_t *} diagnostics  输出
 * @return {*}
 * @author: lkc
 */
void Motor_Sensorless_GetDiagnostics(motor_sensorless_diagnostics_t *diagnostics)
{
    Motor_Sensorless_GetDiagnosticsState(&s_sensorless, diagnostics);
}

/**
 * @description: 判断默认实例观测角是否已接管电流环。
 * @return {bool} true=已 CLOSED
 * @author: lkc
 */
bool Motor_Sensorless_IsClosedLoop(void)
{
    return Motor_Sensorless_IsClosedLoopState(&s_sensorless);
}

/**
 * @description: 判断默认实例开环到观测角接管是否完成。
 * @return {bool} true=blend 已达 1.0
 * @author: lkc
 */
bool Motor_Sensorless_IsTakeoverComplete(void)
{
    return Motor_Sensorless_IsTakeoverCompleteState(&s_sensorless);
}

#ifdef __cplusplus
}
#endif
