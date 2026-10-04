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

void Motor_BldcSixStep_DefaultConfig(motor_bldc_six_step_config_t *config)
{
    memset(config, 0, sizeof(*config));
}
bool Motor_BldcSixStep_Init(motor_bldc_six_step_t *control, const motor_bldc_six_step_config_t *config)
{
    (void)control;
    s_six_init_calls++;
    s_six_config = *config;
    return s_six_init_ok;
}
void Motor_BldcSensorless_DefaultConfig(motor_bldc_sensorless_config_t *config)
{
    memset(config, 0, sizeof(*config));
}
bool Motor_BldcSensorless_Init(motor_bldc_sensorless_t *control, motor_bldc_six_step_t *commutator,
                               const motor_bldc_sensorless_config_t *config)
{
    (void)control;
    (void)commutator;
    s_sensorless_init_calls++;
    s_sensorless_config = *config;
    return s_sensorless_init_ok;
}
void Motor_BldcSixStep_HalApply(const motor_bldc_output_t *output, void *user)
{
    (void)output;
    (void)user;
}

static int Near(float a, float b)
{
    return fabsf(a - b) < 1.0e-7f;
}

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
