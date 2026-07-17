#include "motor_hfi_start.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MOTOR_HFI_START_PI                   (3.14159265358979323846f)
#define MOTOR_HFI_START_TWO_PI               (2.0f * MOTOR_HFI_START_PI)
#define MOTOR_HFI_START_MIN_CURRENT_RESPONSE (1.0e-5f)

static float MotorHfiStart_NormalizeAngle(float angle)
{
    while (angle >= MOTOR_HFI_START_TWO_PI) angle -= MOTOR_HFI_START_TWO_PI;
    while (angle < 0.0f) angle += MOTOR_HFI_START_TWO_PI;
    return angle;
}

void MotorHfiStart_Reset(motor_hfi_start_t *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

void MotorHfiStart_NormalizeConfig(motor_hfi_start_config_t *config)
{
    if (config == NULL) return;
    if (config->voltage < 0.0f) config->voltage = 0.0f;
    if (config->cycles_per_axis < 2U) config->cycles_per_axis = 2U;
    if (config->cycles_per_axis > 32U) config->cycles_per_axis = 32U;
    if (config->min_confidence < 0.0f) config->min_confidence = 0.0f;
    if (config->min_confidence > 1.0f) config->min_confidence = 1.0f;
    if (config->max_bus_voltage_ratio <= 0.0f ||
        config->max_bus_voltage_ratio > 1.0f) {
        config->max_bus_voltage_ratio = 0.20f;
    }
}

static void MotorHfiStart_Finish(motor_hfi_start_t *state,
                                 const motor_hfi_start_config_t *config)
{
    uint16_t i;
    uint16_t best = 0U;
    float maximum = state->response[0];
    float minimum = maximum;

    for (i = 1U; i < MOTOR_HFI_START_AXIS_COUNT; i++) {
        if (state->response[i] > maximum) {
            maximum = state->response[i];
            best = i;
        }
        if (state->response[i] < minimum) minimum = state->response[i];
    }

    state->confidence = maximum > MOTOR_HFI_START_MIN_CURRENT_RESPONSE
        ? (maximum - minimum) / maximum : 0.0f;
    state->angle = MOTOR_HFI_START_PI * (float)best /
                   (float)MOTOR_HFI_START_AXIS_COUNT;
    if (state->negative_response[best] > state->positive_response[best]) {
        state->angle += MOTOR_HFI_START_PI;
    }
    state->angle = MotorHfiStart_NormalizeAngle(state->angle);
    state->succeeded = state->confidence >= config->min_confidence;
    state->running = false;
    state->attempted = true;
}

motor_hfi_start_result_t MotorHfiStart_Step(
    motor_hfi_start_t *state,
    const motor_hfi_start_config_t *config,
    float i_alpha,
    float i_beta,
    float v_bus,
    motor_hfi_start_output_t *output)
{
    float theta;
    float current;
    float delta;
    float voltage;

    if (output != NULL) memset(output, 0, sizeof(*output));
    if (state == NULL || config == NULL || output == NULL ||
        !config->enabled || config->voltage <= 0.0f || state->attempted) {
        return MOTOR_HFI_START_SKIPPED;
    }

    if (!state->running) {
        MotorHfiStart_Reset(state);
        state->running = true;
        state->polarity = 1;
    }

    theta = MOTOR_HFI_START_PI * (float)state->axis /
            (float)MOTOR_HFI_START_AXIS_COUNT;
    current = i_alpha * cosf(theta) + i_beta * sinf(theta);

    if (state->sample_valid) {
        delta = fabsf(current - state->previous_current);
        state->response[state->axis] += delta;
        if (state->polarity > 0) state->positive_response[state->axis] += delta;
        else state->negative_response[state->axis] += delta;
        state->transitions++;
    }

    state->previous_current = current;
    state->sample_valid = true;
    state->polarity = -state->polarity;

    if (state->transitions >= (uint16_t)(config->cycles_per_axis * 2U)) {
        state->axis++;
        state->transitions = 0U;
        state->sample_valid = false;
        state->polarity = 1;
        if (state->axis >= MOTOR_HFI_START_AXIS_COUNT) {
            MotorHfiStart_Finish(state, config);
            return MOTOR_HFI_START_FINISHED;
        }
        theta = MOTOR_HFI_START_PI * (float)state->axis /
                (float)MOTOR_HFI_START_AXIS_COUNT;
    }

    voltage = config->voltage;
    if (voltage > v_bus * config->max_bus_voltage_ratio) {
        voltage = v_bus * config->max_bus_voltage_ratio;
    }
    output->apply_voltage = true;
    output->vd = (float)state->polarity * voltage;
    output->theta = theta;
    return MOTOR_HFI_START_RUNNING;
}
