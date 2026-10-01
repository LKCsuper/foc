/*
 * @Description: 关节阻抗模式（位置、速度、力矩前馈合成电压）
 * @Author: lkc
 *
 * 与 pos / olspeed 分开。无转矩常数时，力矩前馈用 Iq (A)，再按 V/A 换成电压。
 * Vq = Kp·位置误差 + Kd·速度误差 + 力矩前馈 + 慢积分。
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
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 关节阻抗只读状态
 */
typedef struct {
    bool enabled;         /* true=本拍由阻抗公式输出电压 */
    float target_deg;     /* 目标位置 (deg) */
    float position_deg;   /* 当前位置 (deg) */
    float error_deg;      /* 目标减当前位置 (deg) */
    float target_rpm;     /* 目标转速 (RPM) */
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
} motor_joint_state_t;

/**
 * @description: 关节完整命令，供UART/CAN/SPI等适配层一次性提交。
 * @detail: 当前硬件后端为Vq，因此增益和前馈仍使用V；位置和速度统一使用SI单位。
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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
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
 * @description: 推进一拍阻抗公式，电压写入 joint_hold_v。
 * @detail: 转速命令 ol_target_rpm 保持 0，避免速度前馈叠进来。慢积分仅在目标转速接近零、
 *          实测转速不超过 1 RPM 且位置误差不超过 2 deg 时推进。
 * @param {float} dt  调用周期 (s)。慢积分用它累加，大于 0.05 s 时本拍不积分
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_Run(float dt);

/**
 * @description: 复制关节阻抗状态。
 * @param {motor_joint_state_t *} state  输出，为空则忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_Joint_GetState(motor_joint_state_t *state);

/**
 * @description: 校验并整体提交一帧SI单位关节命令。
 * @detail: 所有字段在同一短临界区更新；成功提交会刷新超时计时。当前不写Flash。
 * @return {bool} true=命令合法并已提交
 */
bool Motor_Joint_SubmitCommand(const motor_joint_command_si_t *command);

/**
 * @description: 查询关节阻抗是否使能。
 * @return {bool} true=已使能
 * @author: lkc
 */
bool Motor_Joint_IsEnabled(void);

#ifdef __cplusplus
}
#endif
#endif
