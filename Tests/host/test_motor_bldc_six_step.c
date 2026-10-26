/* BLDC 六步换向：正反转相位、占空比限幅、故障浮空与重复映射拒绝。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motor_bldc_six_step.h"

typedef struct {
    motor_bldc_output_t last;
    unsigned calls;
} output_spy_t;

/**
 * @description: 捕获被测模块本次提交的输出。
 * @param {const motor_bldc_output_t *} output 输出数据
 * @param {void *} user user 参数
 * @return {void} 无
 */
static void capture_output(const motor_bldc_output_t *output, void *user)
{
    output_spy_t *spy = (output_spy_t *)user;
    spy->last = *output;
    spy->calls++;
}

/**
 * @description: 验证正转和反转。
 * @return {void} 无
 */
static void test_forward_and_reverse(void)
{
    motor_bldc_six_step_config_t config;
    motor_bldc_six_step_t control;
    output_spy_t spy;

    memset(&spy, 0, sizeof(spy));
    Motor_BldcSixStep_DefaultConfig(&config);
    config.apply_output = capture_output;
    config.output_user = &spy;
    assert(Motor_BldcSixStep_Init(&control, &config));
    assert(Motor_BldcSixStep_Start(&control, MOTOR_BLDC_DIRECTION_CW, 0.25f));
    assert(Motor_BldcSixStep_Step(&control, 1U, 0.001f));
    assert(spy.last.step == 0U);
    assert(spy.last.phase[0] == MOTOR_BLDC_PHASE_PWM);
    assert(spy.last.phase[1] == MOTOR_BLDC_PHASE_LOW);
    assert(spy.last.phase[2] == MOTOR_BLDC_PHASE_FLOAT);
    assert(spy.last.duty == 0.25f);

    Motor_BldcSixStep_Stop(&control);
    assert(Motor_BldcSixStep_Start(&control, MOTOR_BLDC_DIRECTION_CCW, 0.4f));
    assert(Motor_BldcSixStep_Step(&control, 1U, 0.001f));
    assert(spy.last.phase[0] == MOTOR_BLDC_PHASE_LOW);
    assert(spy.last.phase[1] == MOTOR_BLDC_PHASE_PWM);
    assert(spy.last.phase[2] == MOTOR_BLDC_PHASE_FLOAT);
}

/**
 * @description: 验证序列和占空比限幅。
 * @return {void} 无
 */
static void test_sequence_and_duty_clamp(void)
{
    static const uint8_t hall_sequence[6] = {1U, 5U, 4U, 6U, 2U, 3U};
    motor_bldc_six_step_config_t config;
    motor_bldc_six_step_t control;
    uint8_t step;

    Motor_BldcSixStep_DefaultConfig(&config);
    assert(Motor_BldcSixStep_Init(&control, &config));
    assert(Motor_BldcSixStep_Start(&control, MOTOR_BLDC_DIRECTION_CW, 2.0f));
    for (step = 0U; step < 6U; step++) {
        assert(Motor_BldcSixStep_Step(&control, hall_sequence[step], 0.01f));
        assert(control.output.step == step);
        assert(control.output.duty == 1.0f);
    }
    Motor_BldcSixStep_SetDuty(&control, -1.0f);
    assert(control.output.duty == 0.0f);
}

/**
 * @description: 验证故障高阻全部相。
 * @return {void} 无
 */
static void test_faults_float_all_phases(void)
{
    motor_bldc_six_step_config_t config;
    motor_bldc_six_step_t control;

    Motor_BldcSixStep_DefaultConfig(&config);
    config.commutation_timeout_s = 0.02f;
    assert(Motor_BldcSixStep_Init(&control, &config));
    assert(Motor_BldcSixStep_Start(&control, MOTOR_BLDC_DIRECTION_CW, 0.2f));
    assert(!Motor_BldcSixStep_Step(&control, 0U, 0.001f));
    assert(Motor_BldcSixStep_GetFault(&control) == MOTOR_BLDC_FAULT_INVALID_HALL);
    assert(control.output.phase[0] == MOTOR_BLDC_PHASE_FLOAT);
    assert(control.output.phase[1] == MOTOR_BLDC_PHASE_FLOAT);
    assert(control.output.phase[2] == MOTOR_BLDC_PHASE_FLOAT);
    assert(Motor_BldcSixStep_ClearFault(&control));

    assert(Motor_BldcSixStep_Start(&control, MOTOR_BLDC_DIRECTION_CW, 0.2f));
    assert(Motor_BldcSixStep_Step(&control, 1U, 0.001f));
    assert(Motor_BldcSixStep_Step(&control, 1U, 0.01f));
    assert(!Motor_BldcSixStep_Step(&control, 1U, 0.01f));
    assert(Motor_BldcSixStep_GetFault(&control) == MOTOR_BLDC_FAULT_COMMUTATION_TIMEOUT);
    assert(!Motor_BldcSixStep_IsRunning(&control));
}

/**
 * @description: 验证拒绝重复映射。
 * @return {void} 无
 */
static void test_rejects_duplicate_mapping(void)
{
    motor_bldc_six_step_config_t config;
    motor_bldc_six_step_t control;

    Motor_BldcSixStep_DefaultConfig(&config);
    config.hall_to_step[2] = config.hall_to_step[1];
    assert(!Motor_BldcSixStep_Init(&control, &config));
    assert(Motor_BldcSixStep_GetFault(&control) == MOTOR_BLDC_FAULT_INVALID_CONFIG);
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    test_forward_and_reverse();
    test_sequence_and_duty_clamp();
    test_faults_float_all_phases();
    test_rejects_duplicate_mapping();
    puts("motor_bldc_six_step tests passed");
    return 0;
}
