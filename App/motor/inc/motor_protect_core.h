#ifndef MOTOR_PROTECT_CORE_H
#define MOTOR_PROTECT_CORE_H
#include <stdbool.h>
#include "motor_types.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @description: 保护阈值与去抖时间配置（代码默认，不进 Flash）
 */
typedef struct {
    float over_current;       /* 三相电流峰值上限 A */
    float under_voltage;      /* 运行最低母线电压 V */
    float stall_target_rpm;   /* 启用堵转/失速判据的最低 |目标| RPM */
    float stall_speed_rpm;    /* “几乎静止”判据 |实测| RPM */
    float stall_current;      /* 堵转需同时满足的最低电流峰值 A */
    float speed_loss_ratio;   /* 失速：斜坡到位后，实测/目标 低于此比例 */
    float over_current_time;  /* 过流去抖 s */
    float under_voltage_time; /* 欠压去抖 s */
    float stall_time;         /* 堵转去抖 s */
    float speed_loss_time;    /* 失速去抖 s */
} motor_protect_config_t;

/**
 * @description: 首发故障瞬间快照（停机后仍保留至 ClearFault）
 * @detail: 便于 Shell `olfault` 回看：当时电流、母线、目标/实测转速、无感诊断等
 */
typedef struct {
    float current_peak; /* A */
    float bus_voltage;  /* V */
    float target_rpm;   /* RPM */
    float ramp_rpm;     /* 斜坡当前 RPM */
    float measured_rpm; /* 保护侧速度反馈 RPM */
    float observer_rpm; /* 无感观测机械 RPM */
    float flux;         /* Wb */
    float phase_error;  /* rad */
    float id;           /* A */
    float iq;           /* A */
    float iq_command;   /* 当时 Iq 给定 A */
    float vd_integrator;/* V */
    float vq_integrator;/* V */
    float applied_voltage; /* 无感诊断：实际 αβ 电压模长 V */
    float lock_timer;   /* 无感锁定计时 s */
    float blend;        /* 开环→观测角渐变 0..1 */
    int sensorless_state;
    bool speed_valid;
    bool sensorless_takeover;
    bool speed_loop_enabled;
    bool flux_ok;
    bool observer_speed_ok;
    bool direction_ok;
    bool phase_ok;
} motor_protect_snapshot_t;

/**
 * @description: 锁存故障时由调用者注入的辅助信息
 * @detail: 核心不 #include 无感/速度环。传 NULL 时辅助字段填 0/false，
 *          iq_command 回退为 input->ol_iq_set
 */
typedef struct {
    float observer_rpm; /* RPM */
    float flux;         /* Wb */
    float phase_error;  /* rad */
    float applied_voltage; /* V */
    float lock_timer;   /* s */
    float blend;        /* 0..1 */
    int sensorless_state;
    float iq_command; /* A */
    bool flux_ok;
    bool observer_speed_ok;
    bool direction_ok;
    bool phase_ok;
} motor_protect_aux_t;

/**
 * @description: 保护模块完整状态（配置 + 计时 + 锁存），不持久化
 */
typedef struct {
    motor_protect_config_t config;
    float over_current_timer;  /* s */
    float under_voltage_timer; /* s */
    float stall_timer;         /* s */
    float speed_loss_timer;    /* s */
    float speed_rpm;           /* 外部注入的实测机械 RPM */
    bool speed_valid;
    bool speed_was_reached; /* 失速判据：斜坡到位后是否曾达到目标约 50% */
    bool fault_latched;
    mc_fault_code fault;
    motor_protect_snapshot_t snapshot;
} motor_protect_state_t;
/** 本拍输入快照；电流单位A、电压V、转速为机械RPM。保留旧字段名便于核对映射。
 * 数值须有限，dt须为正；配置须为有效阈值/非负去抖时间。
 * 核心仅锁存原因；上层负责故障后的输出关断及共享状态同步。
 */
typedef struct {
    float ia, ib, ic, v_bus;
    float ol_target_rpm, ol_current_rpm;
    float Id, Iq, ol_vd_int, ol_vq_int, ol_iq_set;
    bool ol_enable, sensorless_takeover, speed_enable;
} motor_protect_input_t;
/** 清零状态；config为空时使用原项目默认阈值。state为空时忽略。 */
void Motor_Protect_InitState(motor_protect_state_t *state, const motor_protect_config_t *config);

/** 清除首因、快照、速度反馈和全部计时，保留配置。 */
void Motor_Protect_ClearFaultState(motor_protect_state_t *state);

/** 注入机械RPM；反馈无效时清堵转/失速计时及曾达速标志。 */
void Motor_Protect_SetSpeedFeedbackState(motor_protect_state_t *state, float rpm, bool valid);

/** 各Check推进自己的去抖计时，返回触发状态，但不锁存故障。输入或状态为空返回false。 */
bool Motor_Protect_CheckOverCurrentInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt);

bool Motor_Protect_CheckUnderVoltageInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt);

bool Motor_Protect_CheckStallInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt);

bool Motor_Protect_CheckSpeedLossInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt);

/** 按过流>欠压>堵转>失速锁存首因和快照，已锁存则不覆盖。
 * aux为空时辅助量清零，Iq命令取输入ol_iq_set。返回是否已锁存。
 */
bool Motor_Protect_RunInputState(motor_protect_state_t *state, const motor_protect_input_t *motor, float dt,
                            const motor_protect_aux_t *aux);

/** 推进一次保护判定与去抖，返回优先故障但不锁存新快照；已锁存则返回首因。
 * 调用者须在同一控制拍内将非NONE结果交给TriggerFaultInputState，不得再重复推进RunInputState。
 */
mc_fault_code Motor_Protect_EvaluateInputState(motor_protect_state_t *state, const motor_protect_input_t *motor,
                                              float dt);

/** 外部触发首因锁存；NONE、空输入或空状态忽略。 */
void Motor_Protect_TriggerFaultInputState(motor_protect_state_t *state, mc_fault_code fault,
                                     const motor_protect_input_t *motor, const motor_protect_aux_t *aux);

bool Motor_Protect_IsFaultLatchedState(const motor_protect_state_t *state);

mc_fault_code Motor_Protect_GetFaultState(const motor_protect_state_t *state);

void Motor_Protect_GetSnapshotState(const motor_protect_state_t *state, motor_protect_snapshot_t *snapshot);
#ifdef __cplusplus
}
#endif
#endif
