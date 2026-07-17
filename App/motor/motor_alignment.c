#include "motor_alignment.h"

#include <stddef.h>
#include <string.h>

void MotorAlignment_Reset(motor_alignment_t *state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->stopped = true;
}

void MotorAlignment_NormalizeConfig(motor_alignment_config_t *config)
{
    if (config == NULL) return;
    if (config->current < 0.0f) config->current = 0.0f;
    if (config->duration < 0.0f) config->duration = 0.0f;
}

void MotorAlignment_MarkComplete(motor_alignment_t *state)
{
    if (state == NULL) return;
    state->stopped = false;
    state->running = false;
    state->complete = true;
}

void MotorAlignment_MarkSkipped(motor_alignment_t *state)
{
    if (state == NULL) return;
    state->stopped = false;
    state->running = false;
    state->complete = false;
}

motor_alignment_result_t MotorAlignment_Step(
    motor_alignment_t *state,
    const motor_alignment_config_t *config,
    float dt,
    motor_alignment_output_t *output)
{
    bool started = false;

    if (output != NULL) memset(output, 0, sizeof(*output));
    if (state == NULL || config == NULL || output == NULL) {
        return MOTOR_ALIGNMENT_SKIPPED;
    }

    if (state->stopped && config->enabled && config->current > 0.0f &&
        config->duration > 0.0f) {
        state->stopped = false;
        state->running = true;
        state->complete = false;
        state->elapsed = 0.0f;
        started = true;
    }
    if (!state->running) return MOTOR_ALIGNMENT_SKIPPED;

    output->hold_position = true;
    output->angle = config->angle;
    output->id_target = config->current;
    output->iq_target = 0.0f;

    if (dt > 0.0f) state->elapsed += dt;
    if (state->elapsed >= config->duration) {
        state->elapsed = config->duration;
        state->running = false;
        state->complete = true;
        output->hold_position = false;
        return MOTOR_ALIGNMENT_FINISHED;
    }
    return started ? MOTOR_ALIGNMENT_STARTED : MOTOR_ALIGNMENT_RUNNING;
}
