/*
 * @Description: 电机保护：过流、欠压、堵转、失速检测与故障锁存
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "motor_param.h"
#include "motor_protect.h"
#include "motor_speed.h"
#include "motor_sensorless.h"

/*
 * 本文件只完成三件事：
 *   1. 分别判断过流、欠压、堵转、失速；
 *   2. 使用“条件持续时间”过滤瞬时毛刺；
 *   3. 锁存第一次故障，等待人工清除。
 *
 * 本文件不会直接修改 PWM 寄存器。调用者 Motor_DmaInt() 收到 true 后，
 * 负责清除控制器运行状态并输出安全占空比，使“故障判断”和“电机停机”解耦。
 */

/*
 * 默认保护参数。
 * 这些值用于当前低压、小电流调试，并不是所有电机都适用：
 *   - 10A：三相中任意一相的瞬时电流上限；
 *   - 8V：有运行命令时的最低母线电压；
 *   - 100RPM：低于该目标转速时不启用速度类保护；
 *   - 30RPM：有转动命令但反馈低于此值，可视为接近静止；
 *   - 0.8A：堵转判断要求电机确实正在带电出力；
 *   - 0.2：实际速度低于目标速度 20% 时进入失速候选状态。
 */
static motor_protect_config_t s_config = {
    .over_current = 10.0f,
    .under_voltage = 8.0f,
    .stall_target_rpm = 100.0f,
    .stall_speed_rpm = 30.0f,
    .stall_current = 0.8f,
    .speed_loss_ratio = 0.2f,
    .over_current_time = 0.0001f,
    .under_voltage_time = 0.1f,
    .stall_time = 1.0f,
    .speed_loss_time = 0.3f,
};

/*
 * 保护运行时状态。
 *
 * 四个 timer 各自独立。例如欠压恢复后只清 under_voltage_timer，
 * 不会把正在累计的过流计时也一起清掉。
 */
static struct {
    float over_current_timer;  /* 过流条件已连续成立的时间，单位 s */
    float under_voltage_timer; /* 欠压条件已连续成立的时间，单位 s */
    float stall_timer;         /* 堵转条件已连续成立的时间，单位 s */
    float speed_loss_timer;    /* 失速条件已连续成立的时间，单位 s */
    float speed_rpm;           /* 外部送入的真实机械转速，单位 RPM */
    bool speed_valid;          /* speed_rpm 是否来自可靠的真实反馈 */
    bool speed_was_reached;    /* 电机是否曾达到目标速度的 50% */
    bool fault_latched;        /* true 表示已经锁存故障，禁止自动恢复 */
    mc_fault_code fault;       /* 第一次触发的故障码 */
    motor_protect_snapshot_t snapshot; /* 第一次故障发生时的数据，停机后仍保留 */
} s_state;

/*
 * 获取三相电流峰值。
 *
 * 电流有正负方向，保护关注的是大小，所以先使用 fabsf() 取绝对值。
 * 假设 ia=-2A、ib=5A、ic=-3A，本函数返回 5A。
 */
static float Motor_Protect_CurrentPeak(const MOTOR_PARAM_S *motor)
{
    /* 先假设 A 相最大，再依次和 B、C 相比较。 */
    float peak = fabsf(motor->ia);

    if (fabsf(motor->ib) > peak) {
        peak = fabsf(motor->ib);
    }
    if (fabsf(motor->ic) > peak) {
        peak = fabsf(motor->ic);
    }

    return peak;
}

/*
 * 对布尔故障条件做时间去抖。
 *
 * 例：dt=0.001s，delay=0.1s，则条件必须连续成立约 100 次才返回 true。
 * 如果第 50 次条件恢复正常，timer 会清零，下一次异常从头开始累计。
 * 因为按 dt 累加而不是按固定次数判断，修改 FOC 频率后保护时间仍然不变。
 */
static bool Motor_Protect_Debounce(bool condition, float dt, float delay, float *timer)
{
    if (!condition) {
        /* 异常不连续，不能继承上一次累计时间。 */
        *timer = 0.0f;
        return false;
    }

    /* 本周期条件仍然异常，将本周期经过的时间加入累计值。 */
    *timer += dt;

    /* 累计时间达到设定延时后，正式确认故障。 */
    return *timer >= delay;
}

/*
 * 锁存首发故障。
 *
 * 例如先发生过流，停机后母线电压又下降产生欠压，fault 仍保留过流，
 * 这样调试人员看到的是最可能的根本原因，而不是停机后的连锁现象。
 */
static void Motor_Protect_LatchFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    if (!s_state.fault_latched) {
        motor_sensorless_diagnostics_t sensorless_diagnostics;

        /*
         * 必须先保存快照再置 fault_latched。下一次 FOC 中断看到锁存后会
         * 立即输出安全状态，届时电流已经下降，无法再还原真正触发值。
         */
        Motor_Sensorless_GetDiagnostics(&sensorless_diagnostics);
        s_state.snapshot.current_peak = Motor_Protect_CurrentPeak(motor);
        s_state.snapshot.bus_voltage = motor->v_bus;
        s_state.snapshot.target_rpm = motor->ol_target_rpm;
        s_state.snapshot.ramp_rpm = motor->ol_current_rpm;
        s_state.snapshot.measured_rpm = s_state.speed_rpm;
        s_state.snapshot.observer_rpm = Motor_Sensorless_GetMechanicalRpm();
        s_state.snapshot.flux = Motor_Sensorless_GetFluxMagnitude();
        s_state.snapshot.phase_error = Motor_Sensorless_GetPhaseError();
        s_state.snapshot.id = motor->Id;
        s_state.snapshot.iq = motor->Iq;
        s_state.snapshot.iq_command =
            motor->speed_enable && Motor_SpeedLoop_IsFeedbackValid() ?
            Motor_SpeedLoop_GetIqTarget() : motor->ol_iq_set;
        s_state.snapshot.vd_integrator = motor->ol_vd_int;
        s_state.snapshot.vq_integrator = motor->ol_vq_int;
        s_state.snapshot.applied_voltage = sensorless_diagnostics.applied_voltage;
        s_state.snapshot.lock_timer = sensorless_diagnostics.lock_timer;
        s_state.snapshot.blend = sensorless_diagnostics.blend;
        s_state.snapshot.sensorless_state = (int)Motor_Sensorless_GetState();
        s_state.snapshot.speed_valid = s_state.speed_valid;
        s_state.snapshot.sensorless_takeover = motor->sensorless_takeover;
        s_state.snapshot.speed_loop_enabled = motor->speed_enable;
        s_state.snapshot.flux_ok = sensorless_diagnostics.flux_ok;
        s_state.snapshot.observer_speed_ok = sensorless_diagnostics.speed_ok;
        s_state.snapshot.direction_ok = sensorless_diagnostics.direction_ok;
        s_state.snapshot.phase_ok = sensorless_diagnostics.phase_ok;
        s_state.fault = fault;
        s_state.fault_latched = true;
    }
}

/**
 * @description: 供无感观测器等安全模块锁存一个已经确认的故障。
 * @detail: 只记录首发故障及其瞬间快照，不在此停 PWM；fault 无效、motor 为空或已有首发故障时忽略，避免覆盖根因。
 * @param {mc_fault_code} fault 要锁存的故障码；FAULT_CODE_NONE 时忽略
 * @param {const MOTOR_PARAM_S} *motor 电机实时数据指针，用于生成故障快照；为空时忽略
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    /* NONE 不是故障；空指针也无法生成可靠快照，因此直接忽略。 */
    if (fault == FAULT_CODE_NONE || motor == NULL) {
        return;
    }

    /* 仍由统一锁存函数保存“第一次”故障，后续故障不会覆盖根因。 */
    Motor_Protect_LatchFault(fault, motor);
}

/**
 * @description: 初始化保护模块配置与运行时状态。
 * @detail: config 为空时使用默认阈值；随后清除全部故障锁存与计时器。
 * @param {const motor_protect_config_t} *config  保护参数；NULL 时使用内置默认值
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_Init(const motor_protect_config_t *config)
{
    if (config != NULL) {
        s_config = *config;
    }
    Motor_Protect_ClearFault();
}

/**
 * @description: 更新堵转/失速保护所用的真实机械转速反馈。
 * @detail: 须传入观测器或传感器测得的真实 RPM，不可使用 ol_current_rpm 开环指令值；
 *          valid 为 false 时清除速度类历史计时。
 * @param {float} rpm     机械转速 (RPM)，正负表示转向
 * @param {bool} valid    转速反馈是否可靠
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid)
{
    s_state.speed_rpm = rpm;
    s_state.speed_valid = valid;
    if (!valid) {
        /* 反馈无效时清除速度类历史，恢复后重新经历完整判定过程。 */
        s_state.speed_was_reached = false;
        s_state.stall_timer = 0.0f;
        s_state.speed_loss_timer = 0.0f;
    }
}

/**
 * @description: 检测三相瞬时电流是否持续超过过流阈值。
 * @detail: 取 max(|ia|, |ib|, |ic|) 与 over_current (A) 比较，经 over_current_time (s) 去抖后确认。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                    本周期时间步长 (s)
 * @return {bool} 去抖后确认过流返回 true
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrent(const MOTOR_PARAM_S *motor, float dt)
{
    /* 任一相电流超过上限并持续 over_current_time 即触发。 */
    bool over_current = Motor_Protect_CurrentPeak(motor) >= s_config.over_current;
    return Motor_Protect_Debounce(over_current, dt, s_config.over_current_time,
                                  &s_state.over_current_timer);
}

/**
 * @description: 检测运行过程中母线电压是否持续低于欠压阈值。
 * @detail: 仅在 ol_enable 且目标转速非零时判断，避免停机和上电滤波阶段误触发。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                    本周期时间步长 (s)
 * @return {bool} 去抖后确认欠压返回 true
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltage(const MOTOR_PARAM_S *motor, float dt)
{
    /* 只在存在运行指令时判断，避免上电滤波建立阶段和停机状态误触发。 */
    bool running = motor->ol_enable && fabsf(motor->ol_target_rpm) > 0.0f;
    bool under_voltage = running && motor->v_bus < s_config.under_voltage;
    return Motor_Protect_Debounce(under_voltage, dt, s_config.under_voltage_time,
                                  &s_state.under_voltage_timer);
}

/**
 * @description: 检测电机是否在带载情况下始终未能起转（堵转）。
 * @detail: 须同时满足：目标转速 ≥ stall_target_rpm (RPM)、真实转速 ≤ stall_speed_rpm (RPM)、
 *          相电流峰值 ≥ stall_current (A)，且速度反馈有效；经 stall_time (s) 去抖确认。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                    本周期时间步长 (s)
 * @return {bool} 去抖后确认堵转返回 true
 * @author: lkc
 */
bool Motor_Protect_CheckStall(const MOTOR_PARAM_S *motor, float dt)
{
    bool commanded;
    bool stopped;
    bool loaded;

    if (!s_state.speed_valid) {
        /* 没有霍尔、编码器或观测器真实反馈时无法可靠判断堵转。 */
        s_state.stall_timer = 0.0f;
        return false;
    }

    /* 有转动指令、转子接近静止且存在负载电流，三者同时成立才算堵转。 */
    commanded = fabsf(motor->ol_target_rpm) >= s_config.stall_target_rpm;
    stopped = fabsf(s_state.speed_rpm) <= s_config.stall_speed_rpm;
    loaded = Motor_Protect_CurrentPeak(motor) >= s_config.stall_current;

    return Motor_Protect_Debounce(commanded && stopped && loaded, dt,
                                  s_config.stall_time, &s_state.stall_timer);
}

/**
 * @description: 检测电机已加速后是否持续严重掉速（失速）。
 * @detail: 须先确认曾达到目标转速 50%，再判断真实转速低于目标 × speed_loss_ratio；
 *          速度反馈无效或目标过低时不启用；经 speed_loss_time (s) 去抖确认。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                    本周期时间步长 (s)
 * @return {bool} 去抖后确认失速返回 true
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLoss(const MOTOR_PARAM_S *motor, float dt)
{
    float target_abs;
    float speed_abs;
    bool speed_lost;

    if (!s_state.speed_valid) {
        /* 没有真实转速反馈时保持失速保护禁用。 */
        s_state.speed_was_reached = false;
        s_state.speed_loss_timer = 0.0f;
        return false;
    }

    target_abs = fabsf(motor->ol_target_rpm);
    speed_abs = fabsf(s_state.speed_rpm);

    /* 目标速度太低时测速误差占比很大，因此不启用失速判断。 */
    if (target_abs < s_config.stall_target_rpm) {
        s_state.speed_was_reached = false;
        s_state.speed_loss_timer = 0.0f;
        return false;
    }

    if (speed_abs >= target_abs * 0.5f) {
        /* 先确认曾经正常加速，避免将启动阶段的低转速误判为失速。 */
        s_state.speed_was_reached = true;
    }

    /* 正常运行后，真实转速持续跌破目标比例才触发失速。 */
    speed_lost = s_state.speed_was_reached &&
                 speed_abs < target_abs * s_config.speed_loss_ratio;
    return Motor_Protect_Debounce(speed_lost, dt, s_config.speed_loss_time,
                                  &s_state.speed_loss_timer);
}

/**
 * @description: 统一保护调度入口，按优先级检测并锁存首发故障。
 * @detail: 已锁存时直接返回 true；否则依次检查过流、欠压、堵转、失速。
 *          返回 true 表示须保持安全输出，不代表本周期新发生故障。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                    本周期时间步长 (s)
 * @return {bool} 故障已锁存或本周期新触发返回 true
 * @author: lkc
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt)
{
    if (s_state.fault_latched) {
        /* 已锁存后不再重复判断，直接要求上层保持停机。 */
        return true;
    }

    /* 按危险程度检查；同一周期多个异常只保留优先级最高的故障。 */
    if (Motor_Protect_CheckOverCurrent(motor, dt)) {                // 查看是否过流
        Motor_Protect_LatchFault(FAULT_CODE_ABS_OVER_CURRENT, motor);
    } else if (Motor_Protect_CheckUnderVoltage(motor, dt)) {        // 是否欠压
        Motor_Protect_LatchFault(FAULT_CODE_UNDER_VOLTAGE, motor);
    } else if (Motor_Protect_CheckStall(motor, dt)) {               // 是否堵转
        Motor_Protect_LatchFault(FAULT_CODE_MOTOR_STALL, motor);
    } else if (Motor_Protect_CheckSpeedLoss(motor, dt)) {           // 是否失速
        Motor_Protect_LatchFault(FAULT_CODE_MOTOR_SPEED_LOSS, motor);
    }

    return s_state.fault_latched;
}

/**
 * @description: 清除全部保护锁存、计时器与速度历史状态。
 * @detail: 仅清除软件状态，不判断硬件故障是否消失；调用前须先停机检查。
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_ClearFault(void)
{
    /* 同时清除锁存、计时器和速度有效状态，复位后从干净状态重新判断。 */
    memset(&s_state, 0, sizeof(s_state));
    s_state.fault = FAULT_CODE_NONE;
}

/**
 * @description: 查询保护模块是否已锁存故障。
 * @return {bool} 已锁存返回 true
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatched(void)
{
    return s_state.fault_latched;
}

/**
 * @description: 获取首次锁存的故障码。
 * @return {mc_fault_code} 首发故障码；无故障时为 FAULT_CODE_NONE
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFault(void)
{
    return s_state.fault;
}

/**
 * @description: 把故障码转换成可读的字符串名称，供 Shell 显示故障原因。
 * @param {mc_fault_code} fault 待转换的故障码
 * @return {*} 故障码对应的可读字符串；无法识别的码返回 "UNKNOWN"
 * @author: lkc
 */
const char *Motor_Protect_GetFaultName(mc_fault_code fault)
{
    switch (fault) {
        case FAULT_CODE_NONE:
            return "NONE";
        case FAULT_CODE_ABS_OVER_CURRENT:
            return "ABS_OVER_CURRENT";
        case FAULT_CODE_UNDER_VOLTAGE:
            return "UNDER_VOLTAGE";
        case FAULT_CODE_MOTOR_STALL:
            return "MOTOR_STALL";
        case FAULT_CODE_MOTOR_SPEED_LOSS:
            return "MOTOR_SPEED_LOSS";
        default:
            return "UNKNOWN";
    }
}

/**
 * @description: 获取首次故障发生瞬间的数据快照。
 * @detail: snapshot 为空指针时不执行任何操作。
 * @param {motor_protect_snapshot_t} *snapshot 接收快照数据的结构体指针
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_GetSnapshot(motor_protect_snapshot_t *snapshot)
{
    if (snapshot != NULL) {
        *snapshot = s_state.snapshot;
    }
}

#ifdef __cplusplus
}
#endif
