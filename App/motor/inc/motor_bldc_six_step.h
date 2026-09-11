/*
 * @Description: 传统 BLDC 霍尔六步换相独立控制器（纯控制逻辑，不依赖 FOC/HAL）
 * @Author: lkc
 */
#ifndef MOTOR_BLDC_SIX_STEP_H
#define MOTOR_BLDC_SIX_STEP_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
/* Private define ------------------------------------------------------------*/
#define MOTOR_BLDC_SIX_STEP_INVALID (0xFFU)
/* Private typedef -----------------------------------------------------------*/
typedef enum {
    MOTOR_BLDC_PHASE_FLOAT = 0, /* 上、下桥臂均关闭 */
    MOTOR_BLDC_PHASE_PWM,       /* 该相作为电源相输出 PWM */
    MOTOR_BLDC_PHASE_LOW,       /* 该相下桥臂常导通 */
} motor_bldc_phase_mode_t;

typedef enum {
    MOTOR_BLDC_DIRECTION_CW = 1,
    MOTOR_BLDC_DIRECTION_CCW = -1,
} motor_bldc_direction_t;

typedef enum {
    MOTOR_BLDC_FAULT_NONE = 0,
    MOTOR_BLDC_FAULT_INVALID_CONFIG,
    MOTOR_BLDC_FAULT_INVALID_HALL,
    MOTOR_BLDC_FAULT_COMMUTATION_TIMEOUT,
} motor_bldc_fault_t;

typedef struct {
    motor_bldc_phase_mode_t phase[3]; /* A/B/C 桥臂命令 */
    float duty;                       /* PWM 相占空比 [0, 1] */
    uint8_t hall;                     /* 产生本命令的霍尔状态 */
    uint8_t step;                     /* 换相步 0..5；停止时为 INVALID */
} motor_bldc_output_t;

typedef void (*motor_bldc_output_fn)(const motor_bldc_output_t *output, void *user);

typedef struct {
    /* 霍尔值(bit0=A, bit1=B, bit2=C)到正转换相步的映射。0/7 必须无效。 */
    uint8_t hall_to_step[8];
    /* 同一有效霍尔状态允许保持的最长时间(s)；<=0 表示关闭超时检测。 */
    float commutation_timeout_s;
    motor_bldc_output_fn apply_output;
    void *output_user;
} motor_bldc_six_step_config_t;

typedef struct {
    motor_bldc_six_step_config_t config;
    motor_bldc_output_t output;
    motor_bldc_direction_t direction;
    motor_bldc_fault_t fault;
    float duty;
    float edge_elapsed_s;
    uint8_t last_hall;
    bool running;
    bool initialized;
} motor_bldc_six_step_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 填充默认霍尔六步换相配置。
 * @detail: 默认正转序列为 001→101→100→110→010→011，换相超时为 0.5 s。
 * @param {motor_bldc_six_step_config_t *} config 输出配置，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSixStep_DefaultConfig(motor_bldc_six_step_config_t *config);

/**
 * @description: 初始化一个独立的霍尔六步换相实例。
 * @detail: 配置非法时返回 false；不访问硬件，桥臂命令通过 apply_output 回调交付。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {const motor_bldc_six_step_config_t *} config 配置，不可为空
 * @return {bool} 初始化成功返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_Init(motor_bldc_six_step_t *control,
                            const motor_bldc_six_step_config_t *config);

/**
 * @description: 启动霍尔六步换相实例。
 * @detail: 启动时先输出三相悬空；首次 Step 收到有效霍尔状态后才输出换相命令。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {motor_bldc_direction_t} direction 转向，只允许 CW 或 CCW
 * @param {float} duty PWM 占空比，自动限制到 [0,1]
 * @return {bool} 成功进入运行态返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_Start(motor_bldc_six_step_t *control,
                             motor_bldc_direction_t direction, float duty);

/**
 * @description: 停止霍尔六步换相实例。
 * @detail: 立即请求三相悬空，清除运行计时但保留故障码供诊断。
 * @param {motor_bldc_six_step_t *} control 控制实例；NULL 时忽略
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSixStep_Stop(motor_bldc_six_step_t *control);

/**
 * @description: 清除六步换相实例的故障状态。
 * @detail: 仅允许在停机状态清除；不会自动重新启动。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @return {bool} 已初始化且成功清除返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_ClearFault(motor_bldc_six_step_t *control);

/**
 * @description: 修改六步换相 PWM 占空比。
 * @detail: 输入自动限制到 [0,1]；运行中会立即重新发布当前换相步。
 * @param {motor_bldc_six_step_t *} control 控制实例；NULL 时忽略
 * @param {float} duty PWM 占空比
 * @return {void} 无
 * @author: lkc
 */
void Motor_BldcSixStep_SetDuty(motor_bldc_six_step_t *control, float duty);

/**
 * @description: 按霍尔状态推进一个六步换相控制周期。
 * @detail: 非法霍尔或同一霍尔保持超时会锁存故障并请求三相悬空。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空
 * @param {uint8_t} hall 原始三位霍尔状态，范围 0..7
 * @param {float} dt_s 本控制周期时间 (s)
 * @return {bool} 本周期产生有效运行输出返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_Step(motor_bldc_six_step_t *control, uint8_t hall, float dt_s);

/**
 * @description: 直接输出指定电气换相步。
 * @detail: 不经过霍尔映射，供无感启动/过零控制器调用；方向沿用 Start 配置。
 * @param {motor_bldc_six_step_t *} control 控制实例，不可为空且须处于运行态
 * @param {uint8_t} step 电气换相步，范围 0..5
 * @return {bool} 已发布有效换相命令返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_CommutateStep(motor_bldc_six_step_t *control, uint8_t step);

/**
 * @description: 获取最近一次六步桥臂输出。
 * @param {const motor_bldc_six_step_t *} control 控制实例，不可为空
 * @return {const motor_bldc_output_t *} 输出只读指针；control 为空时返回 NULL
 * @author: lkc
 */
const motor_bldc_output_t *Motor_BldcSixStep_GetOutput(const motor_bldc_six_step_t *control);

/**
 * @description: 获取六步换相故障码。
 * @param {const motor_bldc_six_step_t *} control 控制实例，不可为空
 * @return {motor_bldc_fault_t} 当前故障；control 为空时返回 INVALID_CONFIG
 * @author: lkc
 */
motor_bldc_fault_t Motor_BldcSixStep_GetFault(const motor_bldc_six_step_t *control);

/**
 * @description: 查询六步换相实例是否正在运行。
 * @param {const motor_bldc_six_step_t *} control 控制实例，不可为空
 * @return {bool} 正在运行返回 true
 * @author: lkc
 */
bool Motor_BldcSixStep_IsRunning(const motor_bldc_six_step_t *control);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /* MOTOR_BLDC_SIX_STEP_H */
