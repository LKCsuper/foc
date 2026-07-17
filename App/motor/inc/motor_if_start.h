#ifndef MOTOR_IF_START_H
#define MOTOR_IF_START_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float min_rpm;
    float phase_boost;
    float min_iq_scale;
} motor_if_start_config_t;

typedef struct {
    float target_rpm;
    float current_rpm;
    bool rotor_position_known;
} motor_if_start_input_t;

typedef struct {
    bool first_cycle;
    float angle_rpm;
    float iq_scale;
    float phase_boost;
} motor_if_start_t;

void MotorIfStart_Reset(motor_if_start_t *state);
void MotorIfStart_NormalizeConfig(motor_if_start_config_t *config);
void MotorIfStart_SetNeutral(motor_if_start_t *state);
void MotorIfStart_SetVfRpm(motor_if_start_t *state, float rpm);
void MotorIfStart_Step(motor_if_start_t *state,
                       const motor_if_start_config_t *config,
                       const motor_if_start_input_t *input);

#ifdef __cplusplus
}
#endif

#endif
