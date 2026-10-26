/*
 * @Description: 关节级联控制（位置/速度外环生成Iq，电流内环生成电压）
 * @Author: lkc
 *
 * 与 pos / olspeed 分开。无转矩常数时，力矩前馈直接使用 Iq (A)。
 * Iq = Kp·位置误差 + Kd·速度误差 + 力矩前馈 + 慢积分，再交给D/Q电流PI。
 * 慢积分只在低速且接近目标时补偿齿槽和摩擦；目标跳变时清零，避免继承上一落点的补偿。
 */
#ifndef MOTOR_JOINT_H
#define MOTOR_JOINT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "motor_config.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum {
    MOTOR_JOINT_BACKEND_VOLTAGE = 0,
    MOTOR_JOINT_BACKEND_TORQUE_CURRENT = 1,
    MOTOR_JOINT_BACKEND_CURRENT_DIRECT = 2,
    MOTOR_JOINT_BACKEND_POSITION_CURRENT = 3,
} motor_joint_backend_t;

/**
 * @description: 关节阻抗只读状态
 */
typedef struct {
    bool enabled;         /* true=本拍由关节级联控制器输出Iq目标 */
    float target_deg;     /* 目标位置 (deg) */
    float position_deg;   /* 当前位置 (deg) */
    float error_deg;      /* 目标减当前位置 (deg) */
    float target_rpm;     /* 目标转速 (RPM) */
    float planned_position_deg; /* ODrive式轨迹当前规划位置 (deg) */
    float planned_rpm;          /* 轨迹当前规划速度 (RPM) */
    bool trajectory_active;     /* true=尚未到达最终位置 */
    float measured_rpm;   /* 实测转速 (RPM) */
    float torque_ff_a;    /* 力矩前馈，用 Iq 表示 (A) */
    float kp_v_per_deg;   /* 刚度 (V/deg) */
    float kd_v_per_rpm;   /* 阻尼 (V/RPM) */
    float ki_v_per_deg_s; /* 慢积分 (V/deg/s)，0=关闭 */
    float bias_v;         /* 慢积分输出 (V)，用来抵消停住时的齿槽和摩擦 */
    float v_limit_v;      /* 输出电压绝对值上限 (V) */
    float output_v;       /* 本拍合成电压 (V) */
    uint32_t command_age_us; /* 最近一次完整命令距今时间 (us) */
    bool timeout_enabled;    /* true=启用命令超时保护 */
    bool command_timeout;    /* true=命令已超时，当前执行阻尼输出 */
    bool speed_valid;        /* true=速度反馈可用于阻尼 */
    bool encoder_fault;      /* true=编码器信号故障 */
    motor_joint_backend_t backend;
    bool hybrid_current_active; /* 混合位置后端当前是否正在使用Iq内环 */
    uint32_t hybrid_enter_count; /* 自上次置零以来进入Iq区次数 */
    uint32_t hybrid_exit_count;  /* 自上次置零以来回到Vq区次数 */
    float hybrid_peak_iq_a;      /* 自上次置零以来Iq区峰值指令 */
    float torque_command_nm; /* 扭矩后端的受限关节输出扭矩 (N*m) */
    float iq_command_a;      /* 扭矩后端换算后的 Iq 目标 (A) */
} motor_joint_state_t;

/**
 * @description: 关节完整命令，供UART/CAN/SPI等适配层一次性提交。
 * @detail: 这是旧电压刻度协议的兼容入口；内部按电机V/A标定换算为Iq后进入电流环。
 */
typedef struct {
    bool enable;
    bool timeout_enable;
    uint32_t timeout_us;       /* 0时使用默认100 ms */
    float position_rad;
    float velocity_rad_s;
    float feedforward_v;
    float kp_v_per_rad;
    float kd_v_per_rad_s;
    float ki_v_per_rad_s;
    float output_limit_v;
} motor_joint_command_si_t;

/**
 * @description: 真实扭矩/Iq后端命令；只有参数完整且有效时才允许启用。
 */
typedef struct {
    bool enable;
    bool timeout_enable;
    uint32_t timeout_us;
    float position_rad;
    float velocity_rad_s;
    float torque_ff_nm;
    float kp_nm_per_rad;
    float kd_nm_per_rad_s;
    float torque_limit_nm;
    float current_limit_a;
    float torque_constant_nm_per_a; /* 电机轴 Kt */
    float gear_ratio;               /* 关节输出扭矩/电机轴扭矩，不得小于1 */
    float efficiency;               /* (0, 1] */
} motor_joint_torque_command_t;

typedef struct {
    bool enable;
    bool timeout_enable;
    uint32_t timeout_us;
    float current_a;
    float current_limit_a;
} motor_joint_current_command_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if MOTOR_CFG_JOINT
/**
 * @description: 用当前累计计数作为零点，并装入默认刚度、阻尼和电压上限。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Init(void);

/**
 * @description: 把当前位置记为零，并清掉目标与前馈。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Zero(void);

/**
 * @description: 开关关节阻抗。
 * @detail: 打开前要求编码器零点有效且无信号故障。关闭时清电压输出。
 * @param {bool} enable  true=使能
 * @return {bool} true=已设置；false=编码器未就绪
 * @author: lkc
 */
bool Motor_Joint_Enable(bool enable);

/**
 * @description: 设置位置、速度和力矩前馈。
 * @param {float} target_deg  相对零点的目标 (deg)
 * @param {float} target_rpm  目标转速 (RPM)
 * @param {float} torque_ff_a  力矩前馈 (A)
 * @detail: 位置目标变化达到 0.01 deg 时清除旧的慢积分，避免换向时旧补偿推错方向。
 * @return {bool} true=三个量都有限且前馈在允许范围内
 * @author: lkc
 */
bool Motor_Joint_SetTarget(float target_deg, float target_rpm, float torque_ff_a);

/**
 * @description: 设置刚度。
 * @param {float} kp_v_per_deg  刚度 (V/deg)，范围 (0, 2]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKp(float kp_v_per_deg);

/**
 * @description: 设置阻尼。
 * @param {float} kd_v_per_rpm  阻尼 (V/RPM)，范围 [0, 0.2]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKd(float kd_v_per_rpm);

/**
 * @description: 设置停住补偿的慢积分。
 * @param {float} ki_v_per_deg_s  积分增益 (V/deg/s)，范围 [0, 1]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetKi(float ki_v_per_deg_s);

/**
 * @description: 设置合成电压绝对值上限。
 * @param {float} limit_v  上限 (V)，范围 [0.2, 6]
 * @return {bool} true=已写入
 * @author: lkc
 */
bool Motor_Joint_SetVoltageLimit(float limit_v);

/**
 * @description: 临时改关节轨迹最高转速，加减速按同样比例缩放。
 * @detail: rpm 小于等于 0 时恢复头文件默认。不写 Flash。
 * @param {float} rpm  最高转速 (RPM)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_SetTrajectoryRpm(float rpm);

/**
 * @description: 读取当前关节轨迹最高转速。
 * @return {float} 最高转速 (RPM)
 * @author: lkc
 */
float Motor_Joint_GetTrajectoryRpm(void);

/**
 * @description: 推进一拍关节外环，Iq目标写入 joint_hold_iq。
 * @detail: 转速命令 ol_target_rpm 保持 0，避免速度前馈叠进来。慢积分仅在目标转速接近零、
 *          实测转速不超过 1 RPM 且位置误差不超过 2 deg 时推进。
 * @param {float} dt  调用周期 (s)。慢积分用它累加，大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Run(float dt);

/**
 * @description: 复制关节阻抗状态。
 * @param {motor_joint_state_t *} state  输出，不可为空，违反约定触发 COMMON_ASSERT
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_GetState(motor_joint_state_t *state);

/**
 * @description: 校验并整体提交一帧SI单位关节命令。
 * @detail: 所有字段在同一短临界区更新；成功提交会刷新超时计时。当前不写Flash。
 * @param {const motor_joint_command_si_t *} command  位置 (rad)、速度 (rad/s)、前馈 (V) 和增益
 * @return {bool} true=命令合法并已提交
 * @author: lkc
 */
bool Motor_Joint_SubmitCommand(const motor_joint_command_si_t *command);

/**
 * @description: 整体提交物理单位扭矩命令，并切换到Iq电流环后端。
 * @detail: torque = ff + kp*位置误差 + kd*速度误差；Iq=torque/(Kt*ratio*efficiency)。
 * @param {const motor_joint_torque_command_t *} command  位置 (rad)、转矩 (N·m) 和 Kt (N·m/A)
 * @return {bool} 参数、编码器状态与限幅全部有效时返回true
 * @author: lkc
 */
bool Motor_Joint_SubmitTorqueCommand(const motor_joint_torque_command_t *command);

/**
 * @description: 提交直接Iq命令，供受限的电流环验证使用。
 * @detail: 不做扭矩换算；必须有有效编码器，且电流不超过显式限制和电机峰值。
 * @param {const motor_joint_current_command_t *} command  目标电流 (A) 和电流上限 (A)
 * @return {bool} true=电流有限且已进入直接电流后端
 * @author: lkc
 */
bool Motor_Joint_SubmitCurrentCommand(const motor_joint_current_command_t *command);

/**
 * @description: 查询关节阻抗是否使能。
 * @return {bool} true=已使能
 * @author: lkc
 */
bool Motor_Joint_IsEnabled(void);
#else
/* 关节关闭：永不使能；关闭请求视为成功，使能请求返回失败。 */
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @return {void} 无
 */
static inline void Motor_Joint_Init(void) {}
/**
 * @description: 开关关节阻抗。
 * @param {bool} enable  true=使能
 * @return {bool} 当前值 !enable
 */
static inline bool Motor_Joint_Enable(bool enable) { return !enable; }
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @param {float} dt 兼容参数，本实现不使用
 * @return {void} 无
 */
static inline void Motor_Joint_Run(float dt) { (void)dt; }
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @return {bool} 本实现固定返回 false
 */
static inline bool Motor_Joint_IsEnabled(void) { return false; }
#endif

#ifdef __cplusplus
}
#endif
#endif
