/* 基于 RL 模型的电流 PI 初值整定；纯计算，不驱动电机、不应用参数。 */
#ifndef MOTOR_AUTOTUNE_H
#define MOTOR_AUTOTUNE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    MOTOR_AUTOTUNE_OK = 0,
    MOTOR_AUTOTUNE_INVALID_INPUT,
    MOTOR_AUTOTUNE_SAMPLE_LIMIT,
    MOTOR_AUTOTUNE_DELAY_LIMIT,
    MOTOR_AUTOTUNE_NUMERIC_ERROR
} motor_autotune_status_t;

typedef struct {
    float resistance_ohm;
    float ld_h;
    float lq_h;
    float control_dt_s;       /* 实际 PI 步进周期，不是 ADC 基准周期 */
    float bandwidth_hz;       /* 请求的模型带宽 Hz */
    float total_delay_s;      /* 总有效延时，包含保持、更新和估计的测量延时，不得重复累加 */
    float min_phase_margin_deg; /* 理想消零模型的延时预算，范围 (0,90) */
} motor_autotune_config_t;

typedef struct {
    float kp;                /* V/A */
    float ki;                /* V/(A*s)，运行时由 PI 乘 dt */
} motor_autotune_pi_t;

typedef struct {
    motor_autotune_pi_t d;
    motor_autotune_pi_t q;
    motor_autotune_pi_t shared; /* 共用增益时以 min(Ld,Lq) 生成 */
    float bandwidth_hz;
    float sample_limit_hz;
    float delay_limit_hz;
    float model_phase_margin_deg;
} motor_autotune_result_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 按 Kp=L*ωc、Ki=R*ωc 生成电流 PI 候选值，可在任务/主机侧调用。
 * @detail: 检查 ωc*dt<=0.2 及理想消零模型 90°-ωc*delay 的裕量预算。
 *          total_delay_s 至少为半个控制周期；未知延时不能填 0 冒充已验证。
 *          检查不代表实际稳定性证明；饱和、滤波、耦合与模型误差需实测验证。
 *          result 不可为空；失败时清空输出。无隐藏状态、不应用增益、不写 Flash。
 * @param {const motor_autotune_config_t *} config 输入配置，不可为空
 * @param {motor_autotune_result_t *} result 输出结果，不可为空
 * @return {motor_autotune_status_t} 成功或具体拒绝原因。
 */
motor_autotune_status_t Motor_Autotune_CurrentPi(const motor_autotune_config_t *config,
                                               motor_autotune_result_t *result);

#ifdef __cplusplus
}
#endif
#endif
