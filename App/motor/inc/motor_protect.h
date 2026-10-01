/*
 * @Description: 电机过流、欠压、堵转与失速保护接口（支持状态实例）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 保护只“判断并锁存原因”，不停 PWM；关断由 ISR/控制出口在故障后执行。
 *   2) 软件保护带去抖，不能代替硬件过流/短路关断。
 *   3) State API 不读无感/速度环全局；锁存诊断经 motor_protect_aux_t 由调用者注入。
 *   4) 无参包装走默认单实例，内部 FillAuxDefault 仍拉无感（过渡期兼容）。
 */
#ifndef MOTOR_PROTECT_H
#define MOTOR_PROTECT_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor_types.h"
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
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
 *          iq_command 回退为 motor->ol_iq_set
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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化指定保护状态对象，写入阈值并清零计时与锁存
 * @param {motor_protect_state_t} *state  状态对象，不可为空
 * @param {const motor_protect_config_t} *config  自定义阈值；NULL 用默认
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_InitState(motor_protect_state_t *state, const motor_protect_config_t *config);

/**
 * @description: 向指定状态注入速度反馈（堵转/失速依赖）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {float} rpm   机械转速 (RPM)，反转为负
 * @param {bool} valid  false 时清堵转/失速计时与“曾达速”标志
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedbackState(motor_protect_state_t *state, float rpm, bool valid);

/**
 * @description: 过流去抖检测（三相电流峰值）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    采样电流来源
 * @param {float} dt  本拍时间 (s)，通常为 ADC 周期
 * @return {bool} true=条件已持续超过 over_current_time
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrentState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 欠压去抖检测（仅在 enable 且目标转速非零时启用）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    母线电压与使能来源
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=欠压已持续超过 under_voltage_time
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltageState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 堵转去抖：高目标 + 近零速 + 大电流，且速度反馈有效
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    目标转速与电流
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=堵转条件已持续超过 stall_time
 * @author: lkc
 */
bool Motor_Protect_CheckStallState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 失速去抖：斜坡跟上目标后，曾达约 50% 再跌破 speed_loss_ratio
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    目标与斜坡转速
 * @param {float} dt  本拍时间 (s)
 * @return {bool} true=失速条件已持续超过 speed_loss_time
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLossState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 按优先级跑完保护并在首次触发时锁存故障与快照
 * @detail: 优先级：过流 > 欠压 > 堵转 > 失速。已锁存则直接返回 true，不覆盖首因
 * @param {motor_protect_state_t} *state  状态对象
 * @param {const MOTOR_PARAM_S} *motor    电机实时量
 * @param {float} dt  本拍时间 (s)
 * @param {const motor_protect_aux_t} *aux  锁存用辅助快照；可为 NULL
 * @return {bool} true=当前已锁存故障（含本拍新锁存）
 * @author: lkc
 */
bool Motor_Protect_RunState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt,
                            const motor_protect_aux_t *aux);

/**
 * @description: 由外部模块直接锁存故障（如无感高速失锁）
 * @param {motor_protect_state_t} *state  状态对象
 * @param {mc_fault_code} fault  故障码；FAULT_CODE_NONE 忽略
 * @param {const MOTOR_PARAM_S} *motor    用于填充快照
 * @param {const motor_protect_aux_t} *aux  辅助快照；可为 NULL
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_TriggerFaultState(motor_protect_state_t *state, mc_fault_code fault,
                                     const MOTOR_PARAM_S *motor, const motor_protect_aux_t *aux);

/**
 * @description: 清除指定状态的故障锁存与计时（保留配置）
 * @param {motor_protect_state_t} *state  状态对象
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_ClearFaultState(motor_protect_state_t *state);

/**
 * @description: 查询指定状态是否已锁存故障
 * @param {const motor_protect_state_t} *state  状态对象；NULL 视为未锁存
 * @return {bool} true=已锁存
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatchedState(const motor_protect_state_t *state);

/**
 * @description: 读取指定状态当前故障码
 * @param {const motor_protect_state_t} *state  状态对象；NULL 返回 NONE
 * @return {mc_fault_code} 故障码
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFaultState(const motor_protect_state_t *state);

/**
 * @description: 拷贝指定状态的故障快照
 * @param {const motor_protect_state_t} *state  状态对象
 * @param {motor_protect_snapshot_t} *snapshot  输出缓冲；NULL 忽略
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_GetSnapshotState(const motor_protect_state_t *state, motor_protect_snapshot_t *snapshot);

/**
 * @description: 初始化默认单实例保护模块
 * @param {const motor_protect_config_t} *config  自定义参数；NULL 用默认
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_Init(const motor_protect_config_t *config);

/**
 * @description: 更新默认实例的速度反馈
 * @param {float} rpm   机械转速 (RPM)
 * @param {bool} valid  反馈是否可信
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid);

/**
 * @description: 默认实例完整保护步进（内部采集无感/速度辅助快照）
 * @param {const MOTOR_PARAM_S} *motor  电机实时参数
 * @param {float} dt  本拍时间 (s)，ADC 路径约 1/30kHz
 * @return {bool} true=已锁存故障
 * @author: lkc
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 默认实例外部触发故障锁存
 * @param {mc_fault_code} fault  故障码
 * @param {const MOTOR_PARAM_S} *motor  用于快照
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor);

/**
 * @description: 清除默认实例故障锁存与计时
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_ClearFault(void);

/**
 * @description: 查询默认实例是否已锁存故障
 * @return {bool} true=已锁存
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatched(void);

/**
 * @description: 读取默认实例当前故障码
 * @return {mc_fault_code} 故障码
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFault(void);

/**
 * @description: 故障码转可读短名（Shell/日志用）
 * @param {mc_fault_code} fault  故障码
 * @return {const char} * 静态字符串，如 "MOTOR_STALL"
 * @author: lkc
 */
const char *Motor_Protect_GetFaultName(mc_fault_code fault);

/**
 * @description: 拷贝默认实例故障快照
 * @param {motor_protect_snapshot_t} *snapshot  输出缓冲
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_GetSnapshot(motor_protect_snapshot_t *snapshot);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif
