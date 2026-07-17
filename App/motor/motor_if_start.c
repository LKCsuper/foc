#include "motor_if_start.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float MotorIfStart_Sign(float value)
{
    return value < 0.0f ? -1.0f : 1.0f;
}

void MotorIfStart_Reset(motor_if_start_t *state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->first_cycle = true;
    state->iq_scale = 1.0f;
}

void MotorIfStart_NormalizeConfig(motor_if_start_config_t *config)
{
    if (config == NULL) return;
    if (config->min_rpm < 0.0f) config->min_rpm = 0.0f;
    if (config->min_iq_scale < 0.0f) config->min_iq_scale = 0.0f;
    if (config->min_iq_scale > 1.0f) config->min_iq_scale = 1.0f;
}

void MotorIfStart_SetNeutral(motor_if_start_t *state)
{
    if (state == NULL) return;
    state->angle_rpm = 0.0f;
    state->iq_scale = 1.0f;
    state->phase_boost = 0.0f;
}

void MotorIfStart_SetVfRpm(motor_if_start_t *state, float rpm)
{
    if (state == NULL) return;
    state->angle_rpm = rpm;
    state->iq_scale = 1.0f;
    state->phase_boost = 0.0f;
}

void MotorIfStart_Step(motor_if_start_t *state,
                       const motor_if_start_config_t *config,
                       const motor_if_start_input_t *input)
{
    float scale;

    if (state == NULL || config == NULL || input == NULL) return;
    state->phase_boost = 0.0f;

    if (state->first_cycle && input->target_rpm != 0.0f) {
        if (!input->rotor_position_known) {
            state->phase_boost = MotorIfStart_Sign(input->target_rpm) *
                                 config->phase_boost;
        }
        state->first_cycle = false;
    }

    state->iq_scale = 1.0f;
    if (!input->rotor_position_known && config->min_rpm > 0.0f &&
        fabsf(input->current_rpm) < config->min_rpm) {
        scale = fabsf(input->current_rpm) / config->min_rpm;
        if (scale < config->min_iq_scale) scale = config->min_iq_scale;
        state->iq_scale = scale;
    }

    if (input->rotor_position_known) {
        state->angle_rpm = input->current_rpm;
    } else if (fabsf(input->target_rpm) >= config->min_rpm &&
               fabsf(input->current_rpm) < config->min_rpm) {
        state->angle_rpm = MotorIfStart_Sign(input->target_rpm) *
                           config->min_rpm;
    } else {
        state->angle_rpm = input->current_rpm;
    }
}
