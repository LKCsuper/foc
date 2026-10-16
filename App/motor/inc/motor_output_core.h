#ifndef MOTOR_OUTPUT_CORE_H
#define MOTOR_OUTPUT_CORE_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @description: 本拍输出仲裁动作（由平台层应用 PWM）
 */
typedef enum {
    MOTOR_CONTROL_ACTION_NONE = 0, /* 无 PWM 更新（极少用） */
    MOTOR_CONTROL_ACTION_SAFE,     /* 三相 mid=period/2；非关桥/非无转矩保证 */
    MOTOR_CONTROL_ACTION_OUTPUT_DQ /* 请求 vd/vq + theta 经反 Park/SVPWM */
} motor_control_action_t;

/**
 * @description: 本拍输出仲裁结果
 */
typedef struct {
    motor_control_action_t action;
    float vd;    /* D 轴电压 (V) */
    float vq;    /* Q 轴电压 (V) */
    float theta; /* 电角度 (rad)，Park/反 Park 用 */
} motor_control_result_t;

/** 逻辑ABC比较值与电流快照；相序交换应在硬件适配层完成。 */
typedef struct {
    uint32_t duty1, duty2, duty3;
    float ia, ib, ic;
} motor_output_sample_t;
/** 故障优先请求SAFE；空请求且无故障返回NONE。SAFE为中点占空比，不是关桥。 */
motor_control_result_t Motor_Control_ArbitrateOutput(bool fault_latched, const motor_control_result_t *proposed);
/** 比较值重建平均alpha/beta电压；period>0，母线为有限正值，死区比例[0,0.5)。
 * 非法输入清零非空输出并返回false；电流仅在比例非零时要求有限。
 */
bool Motor_OutputCore_EstimateVoltage(const motor_output_sample_t *sample, uint32_t period, float v_bus,
                                     float deadtime_fraction, float *alpha, float *beta);
/** 按电流符号减去估计死区压降；指针不可空，参数须有限、母线正、比例合法。
 * 不含管压降，不修改PWM；调用者持有滤波状态，核心不作滤波。
 */
void Motor_OutputCore_SubtractDeadtime(float ia, float ib, float ic, float v_bus,
                                      float fraction, float *alpha, float *beta);
#ifdef __cplusplus
}
#endif
#endif
