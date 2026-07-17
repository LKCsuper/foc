#ifndef MOTOR_HFI_START_H
#define MOTOR_HFI_START_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Standalone zero-speed HFI initial-position estimator.
 *
 * This module has no dependency on the motor parameter structure, PWM driver,
 * current transforms, RTOS, or hardware registers.  The caller supplies the
 * alpha/beta current sample and applies the returned d-axis voltage command.
 */
#define MOTOR_HFI_START_AXIS_COUNT (12U)

typedef struct {
    bool enabled;
    float voltage;
    uint16_t cycles_per_axis;
    float min_confidence;
    float max_bus_voltage_ratio;
} motor_hfi_start_config_t;

typedef struct {
    bool apply_voltage;
    float vd;
    float theta;
} motor_hfi_start_output_t;

typedef enum {
    MOTOR_HFI_START_SKIPPED = 0,
    MOTOR_HFI_START_RUNNING,
    MOTOR_HFI_START_FINISHED
} motor_hfi_start_result_t;

typedef struct {
    bool running;
    bool attempted;
    bool succeeded;
    bool sample_valid;
    uint16_t axis;
    uint16_t transitions;
    int8_t polarity;
    float previous_current;
    float response[MOTOR_HFI_START_AXIS_COUNT];
    float positive_response[MOTOR_HFI_START_AXIS_COUNT];
    float negative_response[MOTOR_HFI_START_AXIS_COUNT];
    float angle;
    float confidence;
} motor_hfi_start_t;

void MotorHfiStart_Reset(motor_hfi_start_t *state);
void MotorHfiStart_NormalizeConfig(motor_hfi_start_config_t *config);
motor_hfi_start_result_t MotorHfiStart_Step(
    motor_hfi_start_t *state,
    const motor_hfi_start_config_t *config,
    float i_alpha,
    float i_beta,
    float v_bus,
    motor_hfi_start_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
