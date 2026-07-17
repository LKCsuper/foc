#ifndef MOTOR_ALIGNMENT_H
#define MOTOR_ALIGNMENT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool enabled;
    float current;
    float duration;
    float angle;
} motor_alignment_config_t;

typedef struct {
    bool stopped;
    bool running;
    bool complete;
    float elapsed;
} motor_alignment_t;

typedef struct {
    bool hold_position;
    float angle;
    float id_target;
    float iq_target;
} motor_alignment_output_t;

typedef enum {
    MOTOR_ALIGNMENT_SKIPPED = 0,
    MOTOR_ALIGNMENT_STARTED,
    MOTOR_ALIGNMENT_RUNNING,
    MOTOR_ALIGNMENT_FINISHED
} motor_alignment_result_t;

void MotorAlignment_Reset(motor_alignment_t *state);
void MotorAlignment_NormalizeConfig(motor_alignment_config_t *config);
void MotorAlignment_MarkSkipped(motor_alignment_t *state);
void MotorAlignment_MarkComplete(motor_alignment_t *state);
motor_alignment_result_t MotorAlignment_Step(
    motor_alignment_t *state,
    const motor_alignment_config_t *config,
    float dt,
    motor_alignment_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
