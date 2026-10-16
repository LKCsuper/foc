#ifndef MOTOR_SPEED_CORE_H
#define MOTOR_SPEED_CORE_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @description: 速度环运行时状态（不持久化）
 */
typedef struct {
    float measured_rpm; /* 反馈机械转速 RPM */
    bool feedback_valid;
    float loop_time; /* 分频累计时间 s */
    float integral;  /* PI 积分 A */
    float error;     /* 速度误差 RPM */
    float iq_target; /* Iq 目标 A */
    float control_target_rpm;   /* 最近一次 PI 计算实际使用的目标 RPM */
    float control_measured_rpm; /* 最近一次 PI 计算实际使用的反馈 RPM */
    uint32_t update_count;
    bool zero_brake_active;    /* 零速制动已锁定进入时的方向 */
    bool zero_brake_released;  /* 已越过零点，在新的非零指令前保持释放 */
    int8_t zero_brake_direction;
} motor_speed_state_t;

/** 最近一次速度 PI 计算帧，附实际计算序号（复位PI不清零）。 */
typedef struct {
    float target_rpm;
    uint32_t update_count;
    float measured_rpm;
    float error;
    float iq_target;
} motor_speed_snapshot_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化指定速度环状态对象，清零反馈、PI 与分频时间
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_Speed_InitState(motor_speed_state_t *state);

/**
 * @description: 更新指定状态对象的机械转速反馈
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {float} rpm 机械转速 (RPM)
 * @param {bool} valid true=反馈可信
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedFeedback_SetState(motor_speed_state_t *state, float rpm, bool valid);

/**
 * @description: 禁用制动时，把最终电压或电流限制成不与转速命令反向。
 * @detail: 速度 PI 本身可以出负修正，用来抵消前馈。本函数用在前馈与 PI 相加之后。
 *          允许制动，或命令转速为 0 时，原样返回。
 * @param {float} command_rpm  转速命令 (RPM)，符号表示驱动方向
 * @param {float} output       即将执行的电压 (V) 或电流 (A)
 * @param {bool} allow_braking true=允许反向制动
 * @return {float} 限幅后的输出，单位与 output 相同
 * @author: lkc
 */
float Motor_Speed_LimitNoBrakeOutput(float command_rpm, float output, bool allow_braking);

/**
 * @description: 清除指定状态对象的 PI（保留反馈转速）
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_ResetState(motor_speed_state_t *state);

/**
 * @description: 预置指定状态对象的 Iq 积分与输出
 * @param {motor_speed_state_t} *state 状态对象，不可为空
 * @param {float} iq 预置 Iq (A)
 * @return {*}
 * @author: lkc
 */
void Motor_SpeedLoop_PrimeIqState(motor_speed_state_t *state, float iq);

/**
 * @description: 读取指定状态当前 Iq 目标
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 0
 * @return {float} Iq 目标 (A)
 * @author: lkc
 */
float Motor_SpeedLoop_GetIqTargetState(const motor_speed_state_t *state);

/**
 * @description: 读取指定状态实测机械转速
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 0
 * @return {float} 机械转速 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetMeasuredRpmState(const motor_speed_state_t *state);

/**
 * @description: 读取指定状态速度误差（目标−实测）
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 0
 * @return {float} 速度误差 (RPM)
 * @author: lkc
 */
float Motor_SpeedLoop_GetErrorState(const motor_speed_state_t *state);

/**
 * @description: 查询指定状态速度反馈是否可信
 * @param {const motor_speed_state_t} *state  状态对象；NULL 返回 false
 * @return {bool} true=反馈可信，可用于闭环
 * @author: lkc
 */
bool Motor_SpeedLoop_IsFeedbackValidState(const motor_speed_state_t *state);

/** 每拍明确输入；RPM为机械转速，Iq单位A，kp为A/RPM，ki为A/(RPM*s)。
 * command_rpm是原始命令，target_rpm是调用者完成斜坡后的目标。
 * 数值须有限，dt须为正；loop_hz小于1时核心按1Hz计算，不修改输入。
 */
typedef struct {
    bool enabled;       /* 上层已允许速度闭环 */
    bool inhibited;     /* 其他控制路径占用输出，复位速度PI */
    bool allow_braking; /* 零速是否允许主动制动 */
    bool position_hold; /* 位置保持时零速是控制目标，不释放制动 */
    float command_rpm, target_rpm, loop_hz, kp, ki, iq_limit;
} motor_speed_input_t;
/** 返回true表示进入分频处理（不一定已计算PI）；false表示禁用、反馈无效或零速释放。
 * 调用者持有状态并负责跨任务同步，核心不实现锁或转速斜坡。
 */
bool Motor_SpeedLoop_StepState(motor_speed_state_t *state, const motor_speed_input_t *input, float dt);
/** 外部已定时的调用入口：每次允许控制时计算一次PI，dt为实际间隔；不再内部周期分频。 */
bool Motor_SpeedLoop_StepScheduledState(motor_speed_state_t *state, const motor_speed_input_t *input, float dt);

#ifdef __cplusplus
}
#endif
#endif
