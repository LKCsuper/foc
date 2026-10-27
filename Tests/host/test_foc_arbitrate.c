/* FOC 最终输出仲裁：控制结果生成后出现故障时仍须复位并输出 SAFE。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "common_assert.h"
#include "motor_control.h"
#include "motor_output.h"
#include "motor_feedback.h"
#include "motor_protect.h"

static motor_control_t s_control;
static bool s_latched;
static unsigned s_resets, s_outputs;
static motor_feedback_source_t s_source;
static motor_control_result_t s_applied;
static motor_output_path_t s_path;

/**
 * @description: 模拟输出提交前的最终故障查询。
 * @return {bool} 当前测试设定的锁存状态
 */
bool Motor_Protect_IsFaultLatched(void) { return s_latched; }

/**
 * @description: 检查故障路径复位了当前实例。
 * @param {motor_control_t *} ctrl 当前控制实例，不可为空
 * @param {MOTOR_PARAM_S *} motor 当前电机，不可为空
 * @return {void} 无
 */
void Motor_Control_ResetRuntime(motor_control_t *ctrl, MOTOR_PARAM_S *motor)
{
    COMMON_ASSERT(ctrl != NULL && motor != NULL);
    assert(ctrl == &s_control);
    s_resets++;
}

/**
 * @description: 提供当前测试反馈源。
 * @return {motor_feedback_source_t} 当前反馈源
 */
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void) { return s_source; }

/**
 * @description: 记录最终输出请求，检查故障后未提交主动电压。
 * @param {MOTOR_PARAM_S *} motor 电机实例，不可为空
 * @param {float} v_bus 母线电压 (V)
 * @param {const motor_control_result_t *} result 仲裁后的结果，不可为空
 * @param {motor_output_path_t} path 输出路径
 * @return {void} 无
 */
void Motor_OutputApplyControlResultForPath(MOTOR_PARAM_S *motor, float v_bus,
                                         const motor_control_result_t *result, motor_output_path_t path)
{
    COMMON_ASSERT(motor != NULL && result != NULL);
    assert(v_bus == motor->v_bus);
    assert(!s_latched || result->action == MOTOR_CONTROL_ACTION_SAFE);
    s_applied = *result;
    s_path = path;
    s_outputs++;
}

#include "foc_arbitrate_under_test.inc"

/**
 * @description: 对照公共仲裁语义，覆盖最终故障、全部候选动作及输出路径。
 * @return {int} 所有断言通过返回 0
 */
int main(void)
{
    MOTOR_PARAM_S motor = {0};
    motor.v_bus = 24.0f;
    for (unsigned mode = 0; mode < 2; mode++) {
        motor.ol_mode = mode ? MOTOR_OL_MODE_IF : MOTOR_OL_MODE_VF;
        for (unsigned source = 0; source < 5; source++) {
            s_source = (motor_feedback_source_t)source;
            for (unsigned action = 0; action < 4; action++) {
                for (unsigned fault = 0; fault < 2; fault++) {
                    motor_control_result_t proposed = {(motor_control_action_t)action, 1.0f, -2.0f, 0.5f};
                    motor_control_result_t original = proposed;
                    motor_control_result_t expected;
                    motor_output_path_t path = MOTOR_OUTPUT_PATH_VF;
                    s_latched = fault != 0;
                    expected = Motor_Control_ArbitrateOutput(s_latched, &proposed);
                    s_resets = s_outputs = 0;
                    Motor_FocIsr_ArbitrateAndApply(&motor, proposed);
                    assert(s_resets == fault && s_outputs == 1);
                    assert(memcmp(&expected, &s_applied, sizeof(expected)) == 0);
                    assert(memcmp(&original, &proposed, sizeof(original)) == 0);
                    if (mode) {
                        if (s_source == MOTOR_FEEDBACK_SRC_OPENLOOP || s_source == MOTOR_FEEDBACK_SRC_SENSORLESS)
                            path = MOTOR_OUTPUT_PATH_SENSORLESS_IF;
                        else if (s_source == MOTOR_FEEDBACK_SRC_ENCODER ||
                                 (s_source == MOTOR_FEEDBACK_SRC_HALL && MOTOR_HALL_USE_VESC_SVM))
                            path = MOTOR_OUTPUT_PATH_SENSORED_IF;
                    }
                    assert(s_path == path);
                }
            }
        }
    }
    puts("FOC final arbitration: late fault, actions and output paths passed");
    return 0;
}
