/* BLDC 默认配置：六步与无感初始化及启动/检测参数默认值。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_bldc_defaults.h"
#include "motor_bldc_six_step_hal.h"

static bool s_six_init_ok = true;
static bool s_sensorless_init_ok = true;
static unsigned s_six_init_calls;
static unsigned s_sensorless_init_calls;
static motor_bldc_six_step_config_t s_six_config;
static motor_bldc_sensorless_config_t s_sensorless_config;

/**
 * @description: 主机测试替身：填充默认霍尔六步换相配置。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_bldc_six_step_config_t *} config 配置
 * @return {void} 无
 */
void Motor_BldcSixStep_DefaultConfig(motor_bldc_six_step_config_t *config)
{
    memset(config, 0, sizeof(*config));
}
/**
 * @description: 主机测试替身：初始化一个独立的霍尔六步换相实例。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_bldc_six_step_t *} control 兼容参数，本实现不使用
 * @param {const motor_bldc_six_step_config_t *} config 配置
 * @return {bool} 测试记录值 s_six_init_ok
 */
bool Motor_BldcSixStep_Init(motor_bldc_six_step_t *control, const motor_bldc_six_step_config_t *config)
{
    (void)control;
    s_six_init_calls++;
    s_six_config = *config;
    return s_six_init_ok;
}
/**
 * @description: 主机测试替身：填充无感六步控制默认配置。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_bldc_sensorless_config_t *} config 配置
 * @return {void} 无
 */
void Motor_BldcSensorless_DefaultConfig(motor_bldc_sensorless_config_t *config)
{
    memset(config, 0, sizeof(*config));
}
/**
 * @description: 主机测试替身：初始化一个独立的无感六步控制实例。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_bldc_sensorless_t *} control 兼容参数，本实现不使用
 * @param {motor_bldc_six_step_t *} commutator 兼容参数，本实现不使用
 * @param {const motor_bldc_sensorless_config_t *} config 配置
 * @return {bool} 测试记录值 s_sensorless_init_ok
 */
bool Motor_BldcSensorless_Init(motor_bldc_sensorless_t *control, motor_bldc_six_step_t *commutator,
                               const motor_bldc_sensorless_config_t *config)
{
    (void)control;
    (void)commutator;
    s_sensorless_init_calls++;
    s_sensorless_config = *config;
    return s_sensorless_init_ok;
}
/**
 * @description: 主机测试替身：将六步控制结果应用到平台无关电机 HAL。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const motor_bldc_output_t *} output 兼容参数，本实现不使用
 * @param {void *} user 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_BldcSixStep_HalApply(const motor_bldc_output_t *output, void *user)
{
    (void)output;
    (void)user;
}

/**
 * @description: 判断两浮点值是否在测试容差内。
 * @param {float} a a 参数
 * @param {float} b b 参数
 * @return {int} 本次计算或查询的结果
 */
static int Near(float a, float b)
{
    return fabsf(a - b) < 1.0e-7f;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_bldc_six_step_t commutator;
    motor_bldc_sensorless_t sensorless;

    memset(&commutator, 0, sizeof(commutator));
    memset(&sensorless, 0, sizeof(sensorless));
    assert(Motor_BldcDefaults_Init(&commutator, &sensorless));
    assert(s_six_init_calls == 1U && s_sensorless_init_calls == 1U);
    assert(s_six_config.apply_output == Motor_BldcSixStep_HalApply && s_six_config.output_user == NULL);
    assert(Near(s_sensorless_config.startup_duty, 0.08f));
    assert(Near(s_sensorless_config.startup_final_period_s, 0.012f));
    assert(Near(s_sensorless_config.bemf_integral_threshold_vs, 0.004f));
    assert(s_sensorless_config.duty_period_enabled && s_sensorless_config.step0_rising);
    assert(!s_sensorless_config.integral_auto_detect_enabled);

    s_six_init_ok = false;
    assert(!Motor_BldcDefaults_Init(&commutator, &sensorless));
    assert(s_sensorless_init_calls == 1U);
    assert(!Motor_BldcDefaults_Init(NULL, &sensorless));
    puts("test_motor_bldc_defaults: pass");
    return 0;
}
