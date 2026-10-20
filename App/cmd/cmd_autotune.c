/* 自动整定应用入口：仅展示候选值，不启动检测、不应用或保存参数。 */
#include "motor_autotune.h"
#include "motor.h"
#include "motor_detect.h"
#include "motor_timing.h"
#include "hal_motor.h"
#include "printf_color.h"
#include <math.h>
#include <stdlib.h>
#if MOTOR_CFG_CMD_IDENTIFY

static bool Autotune_ParsePositive(const char *text, float *value)
{
    char *end;
    if (text == NULL) return false;
    *value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(*value) && *value > 0.0f;
}

/** 任务侧命令；延时必须由用户显式输入，不把默认周期数当作硬件测量值。 */
int Cmd_Autotune(int argc, char *argv[])
{
    motor_command_t command;
    motor_status_t status;
    motor_detect_result_t detected;
    motor_autotune_config_t config = {0};
    motor_autotune_result_t candidate;
    motor_autotune_status_t result;
    float delay_us, adc_hz;

    if (argc < 1 || argv == NULL || argc < 3 || argc > 4 ||
        !Autotune_ParsePositive(argv[1], &config.bandwidth_hz) ||
        !Autotune_ParsePositive(argv[2], &delay_us) ||
        (argc == 4 && !Autotune_ParsePositive(argv[3], &config.min_phase_margin_deg))) {
        PRINTF("Usage: pitune <bandwidth_Hz> <total_delay_us> [min_margin_deg=60]\n");
        PRINTF("Preview only; run detect rl first. Delay must include hold/update/measurement.\n");
        return -1;
    }
    if (argc == 3) {
        config.min_phase_margin_deg = 60.0f;
    }
    if (Motor_GetCommand(&motor, &command) != MOTOR_OK || Motor_GetStatus(&motor, &status) != MOTOR_OK ||
        command.enable || status.enable || status.fault_latched || status.fault != 0 || Motor_Detect_IsActive()) {
        PRINTF("pitune rejected: stop motor, finish detection and clear faults first.\n");
        return -1;
    }
    DETECT_GetResult(&detected);
    if (detected.version != MOTOR_DETECT_RESULT_VERSION || !detected.resistance_valid ||
        !detected.inductance_valid || detected.error != MOTOR_DETECT_ERROR_NONE || detected.fault != 0) {
        PRINTF("pitune rejected: no successful R/L result; run detect rl on this motor first.\n");
        return -1;
    }
    adc_hz = Hal_Motor_AdcSampleRateHz();
    if (!isfinite(adc_hz) || adc_hz <= 0.0f) {
        PRINTF("pitune rejected: HAL sample frequency unavailable.\n");
        return -1;
    }
    config.control_dt_s = (float)FOC_CONTROL_LOOP_FREQ_DIVIDER / adc_hz;
    config.total_delay_s = delay_us * 1.0e-6f;
    config.resistance_ohm = detected.resistance_ohm;
    config.ld_h = detected.inductance_d_h;
    config.lq_h = detected.inductance_q_h;
    result = Motor_Autotune_CurrentPi(&config, &candidate);
    if (result != MOTOR_AUTOTUNE_OK) {
        const char *reason = "invalid input/model";
        if (result == MOTOR_AUTOTUNE_SAMPLE_LIMIT) {
            reason = "bandwidth exceeds sample budget";
        } else if (result == MOTOR_AUTOTUNE_DELAY_LIMIT) {
            reason = "bandwidth exceeds delay/margin budget";
        } else if (result == MOTOR_AUTOTUNE_NUMERIC_ERROR) {
            reason = "numeric range exceeded";
        }
        PRINTF("pitune rejected: %s\n", reason);
        return -1;
    }
    PRINTF("PI candidate only: bw=%.2f Hz dt=%.3f us delay=%.3f us\n",
           (double)candidate.bandwidth_hz, (double)(config.control_dt_s * 1.0e6f), (double)delay_us);
    PRINTF("Source: latest RAM R/L, I_test=%.3f A; rerun detection after changing motor.\n",
           (double)detected.test_current_a);
    PRINTF("R=%.6f ohm Ld=%.3f uH Lq=%.3f uH\n", (double)config.resistance_ohm,
           (double)(config.ld_h * 1.0e6f), (double)(config.lq_h * 1.0e6f));
    PRINTF("d: Kp=%.6f Ki=%.6f; q: Kp=%.6f Ki=%.6f\n", (double)candidate.d.kp,
           (double)candidate.d.ki, (double)candidate.q.kp, (double)candidate.q.ki);
    PRINTF("shared candidate: Kp=%.6f Ki=%.6f (Ki not multiplied by dt)\n",
           (double)candidate.shared.kp, (double)candidate.shared.ki);
    PRINTF("Budget: sample<=%.2f Hz delay<=%.2f Hz; ideal axis margin=%.2f deg (not measured)\n",
           (double)candidate.sample_limit_hz, (double)candidate.delay_limit_hz,
           (double)candidate.model_phase_margin_deg);
    PRINTF("Shared gains need both-axis validation. No gains applied or saved.\n");
    return 0;
}
#endif /* MOTOR_CFG_CMD_IDENTIFY */
