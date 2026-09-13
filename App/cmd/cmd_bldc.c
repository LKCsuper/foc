/*
 * @Description: BLDC 六步换相测试命令
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "includes.h"
#include "motor_it.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static const char *Cmd_BldcModeName(motor_bldc_sensorless_mode_t mode);
static const char *Cmd_BldcFaultName(motor_bldc_sensorless_fault_t fault);
static void Cmd_BldcPrintStatus(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 将 BLDC 无感状态转换为可读字符串。
 * @param {motor_bldc_sensorless_mode_t} mode 无感状态
 * @return {const char *} 状态名称
 * @author: lkc
 */
static const char *Cmd_BldcModeName(motor_bldc_sensorless_mode_t mode)
{
    switch (mode) {
        case MOTOR_BLDC_SENSORLESS_ALIGN:
            return "ALIGN";
        case MOTOR_BLDC_SENSORLESS_OPEN_LOOP:
            return "OPEN_LOOP";
        case MOTOR_BLDC_SENSORLESS_OPEN_LOOP_FIXED:
            return "OPEN_FIXED";
        case MOTOR_BLDC_SENSORLESS_CLOSED_LOOP:
            return "CLOSED_LOOP";
        case MOTOR_BLDC_SENSORLESS_FAULT:
            return "FAULT";
        case MOTOR_BLDC_SENSORLESS_IDLE:
        default:
            return "IDLE";
    }
}

/**
 * @description: 将 BLDC 无感故障转换为可读字符串。
 * @param {motor_bldc_sensorless_fault_t} fault 无感故障码
 * @return {const char *} 故障名称
 * @author: lkc
 */
static const char *Cmd_BldcFaultName(motor_bldc_sensorless_fault_t fault)
{
    switch (fault) {
        case MOTOR_BLDC_SENSORLESS_FAULT_INVALID_CONFIG:
            return "INVALID_CONFIG";
        case MOTOR_BLDC_SENSORLESS_FAULT_COMMUTATOR:
            return "COMMUTATOR";
        case MOTOR_BLDC_SENSORLESS_FAULT_STARTUP_TIMEOUT:
            return "STARTUP_TIMEOUT";
        case MOTOR_BLDC_SENSORLESS_FAULT_BEMF_LOST:
            return "BEMF_LOST";
        case MOTOR_BLDC_SENSORLESS_FAULT_INVALID_SAMPLE:
            return "INVALID_SAMPLE";
        case MOTOR_BLDC_SENSORLESS_FAULT_NONE:
        default:
            return "NONE";
    }
}

/**
 * @description: 打印默认 BLDC 实例状态。
 * @return {void} 无
 * @author: lkc
 */
static void Cmd_BldcPrintStatus(void)
{
    motor_bldc_status_t status;
    uint8_t step;

    if (!Motor_Bldc_GetStatus(&status)) {
        PRINTF("BLDC status unavailable\r\n");
        return;
    }
    PRINTF("BLDC selected=%d running=%d mode=%s fault=%s motor_fault=%u\r\n",
           status.selected, status.running, Cmd_BldcModeName(status.mode),
           Cmd_BldcFaultName(status.fault), (unsigned)status.motor_fault);
    PRINTF("  dir=%s duty=%.3f step=%u commutations=%lu forced=%lu armed=%lu\r\n",
           status.direction == MOTOR_BLDC_DIRECTION_CCW ? "CCW" : "CW", status.duty,
           (unsigned)status.step, (unsigned long)status.integral_commutations,
           (unsigned long)status.forced_commutations, (unsigned long)status.armed_sectors);
    PRINTF("  bemf=%.3fV range=[%.3f,%.3f]V sector=%.3fms period=%.3fms vbus=%.2fV\r\n",
           status.bemf_v, status.bemf_min_v, status.bemf_max_v,
           status.sector_elapsed_s * 1000.0f, status.commutation_period_s * 1000.0f, motor.v_bus);
    PRINTF("  period_range=[%.3f,%.3f]ms\r\n",
           status.minimum_commutation_period_s * 1000.0f,
           status.maximum_commutation_period_s * 1000.0f);
    PRINTF("  sense=INTEGRAL integral=%.6f/%.6fV*s learned=%u\r\n",
           status.bemf_integral_vs, status.bemf_integral_threshold_vs,
           status.startup_integral_count);
    PRINTF("  threshold/step=%.6f,%.6f,%.6f,%.6f,%.6f,%.6fV*s\r\n",
           status.step_integral_threshold_vs[0], status.step_integral_threshold_vs[1],
           status.step_integral_threshold_vs[2], status.step_integral_threshold_vs[3],
           status.step_integral_threshold_vs[4], status.step_integral_threshold_vs[5]);
    PRINTF("  adc_sample=TIM2:%lu (TIM1:%lu)\r\n",
           (unsigned long)status.bemf_sample_count,
           (unsigned long)(2U * status.bemf_sample_count));
    PRINTF("  armed/step=%u,%u,%u,%u,%u,%u\r\n", (unsigned)status.step_armed_sectors[0],
           (unsigned)status.step_armed_sectors[1], (unsigned)status.step_armed_sectors[2],
           (unsigned)status.step_armed_sectors[3], (unsigned)status.step_armed_sectors[4],
           (unsigned)status.step_armed_sectors[5]);
    PRINTF("  integral_comm/step=%u,%u,%u,%u,%u,%u\r\n",
           (unsigned)status.step_integral_commutations[0],
           (unsigned)status.step_integral_commutations[1],
           (unsigned)status.step_integral_commutations[2],
           (unsigned)status.step_integral_commutations[3],
           (unsigned)status.step_integral_commutations[4],
           (unsigned)status.step_integral_commutations[5]);
    PRINTF("  forced_comm/step=%u,%u,%u,%u,%u,%u\r\n",
           (unsigned)status.step_forced_commutations[0],
           (unsigned)status.step_forced_commutations[1],
           (unsigned)status.step_forced_commutations[2],
           (unsigned)status.step_forced_commutations[3],
           (unsigned)status.step_forced_commutations[4],
           (unsigned)status.step_forced_commutations[5]);
    PRINTF("  last_period/step=%.3f,%.3f,%.3f,%.3f,%.3f,%.3fms\r\n",
           status.step_last_period_s[0] * 1000.0f, status.step_last_period_s[1] * 1000.0f,
           status.step_last_period_s[2] * 1000.0f, status.step_last_period_s[3] * 1000.0f,
           status.step_last_period_s[4] * 1000.0f, status.step_last_period_s[5] * 1000.0f);
    PRINTF("  zero_cross/step=%u,%u,%u,%u,%u,%u\r\n",
           (unsigned)status.step_zero_crossings[0], (unsigned)status.step_zero_crossings[1],
           (unsigned)status.step_zero_crossings[2], (unsigned)status.step_zero_crossings[3],
           (unsigned)status.step_zero_crossings[4], (unsigned)status.step_zero_crossings[5]);
    PRINTF("  zero_time/step=%.3f,%.3f,%.3f,%.3f,%.3f,%.3fms\r\n",
           status.step_zero_cross_time_s[0] * 1000.0f,
           status.step_zero_cross_time_s[1] * 1000.0f,
           status.step_zero_cross_time_s[2] * 1000.0f,
           status.step_zero_cross_time_s[3] * 1000.0f,
           status.step_zero_cross_time_s[4] * 1000.0f,
           status.step_zero_cross_time_s[5] * 1000.0f);
    PRINTF("  min/step=%.3f,%.3f,%.3f,%.3f,%.3f,%.3fV\r\n", status.step_bemf_min_v[0],
           status.step_bemf_min_v[1], status.step_bemf_min_v[2], status.step_bemf_min_v[3],
           status.step_bemf_min_v[4], status.step_bemf_min_v[5]);
    PRINTF("  max/step=%.3f,%.3f,%.3f,%.3f,%.3f,%.3fV\r\n", status.step_bemf_max_v[0],
           status.step_bemf_max_v[1], status.step_bemf_max_v[2], status.step_bemf_max_v[3],
           status.step_bemf_max_v[4], status.step_bemf_max_v[5]);
    for (step = 0U; step < 6U; step++) {
        PRINTF("  bemf_wave/%u=%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3fV\r\n",
               (unsigned)step, status.step_bemf_waveform_v[step][0],
               status.step_bemf_waveform_v[step][1], status.step_bemf_waveform_v[step][2],
               status.step_bemf_waveform_v[step][3], status.step_bemf_waveform_v[step][4],
               status.step_bemf_waveform_v[step][5], status.step_bemf_waveform_v[step][6],
               status.step_bemf_waveform_v[step][7]);
    }
}

/**
 * @description: bldc 命令入口。
 * @detail: 支持自动无感启动和固定扇区周期开环诊断；启动占空比限制为 (0,0.25]。
 * @param {int} argc 参数数量
 * @param {char *[]} argv 参数数组
 * @return {int} 命令处理结果，0 表示已处理
 * @author: lkc
 */
int Cmd_Bldc(int argc, char *argv[])
{
    motor_bldc_direction_t direction = MOTOR_BLDC_DIRECTION_CW;
    float duty;
    float sector_period_ms;
    int direction_index;

    if (argc < 2 || strcmp(argv[1], "status") == 0) {
        Cmd_BldcPrintStatus();
        return 0;
    }
    if (strcmp(argv[1], "stop") == 0) {
        Motor_Bldc_Stop();
        PRINTF("BLDC stopped and still selected; use 'bldc exit' before FOC\r\n");
        return 0;
    }
    if (strcmp(argv[1], "exit") == 0) {
        Motor_Bldc_Exit();
        PRINTF("BLDC exited; FOC remains stopped\r\n");
        return 0;
    }
    if (strcmp(argv[1], "tune") == 0) {
        float integral_mvs;
        float minimum_period_ms;

        if (argc < 4) {
            PRINTF("Usage: bldc tune <integral_mV_s> <minimum_period_ms>\r\n");
            return 0;
        }
        integral_mvs = (float)atof(argv[2]);
        minimum_period_ms = (float)atof(argv[3]);
        if (!Motor_Bldc_SetTuning(integral_mvs * 0.001f,
                                  minimum_period_ms * 0.001f)) {
            PRINTF("BLDC tune rejected: stop first; integral=[0.05,10]mV*s, period=[0.5,100]ms\r\n");
            return 0;
        }
        PRINTF("BLDC tune: integral=%.3fmV*s minimum_period=%.3fms\r\n",
               integral_mvs, minimum_period_ms);
        return 0;
    }
    if ((strcmp(argv[1], "start") != 0 && strcmp(argv[1], "open") != 0) || argc < 3) {
        PRINTF("Usage: bldc start <duty> [cw|ccw] | open <duty> <sector_ms> [cw|ccw]\r\n");
        PRINTF("       bldc tune <integral_mV_s> <minimum_period_ms>\r\n");
        PRINTF("       bldc stop | exit | status\r\n");
        return 0;
    }

    duty = (float)atof(argv[2]);
    direction_index = strcmp(argv[1], "open") == 0 ? 4 : 3;
    if (argc > direction_index) {
        if (strcmp(argv[direction_index], "ccw") == 0) {
            direction = MOTOR_BLDC_DIRECTION_CCW;
        } else if (strcmp(argv[direction_index], "cw") != 0) {
            PRINTF("Direction must be cw or ccw\r\n");
            return 0;
        }
    }
    if (strcmp(argv[1], "open") == 0) {
        if (argc < 4) {
            PRINTF("Usage: bldc open <duty> <sector_ms> [cw|ccw]\r\n");
            return 0;
        }
        sector_period_ms = (float)atof(argv[3]);
        if (!Motor_Bldc_StartOpenLoop(direction, duty, sector_period_ms * 0.001f)) {
            PRINTF("BLDC open rejected: duty=(0,0.25], sector_ms=[0.5,500]\r\n");
            return 0;
        }
        PRINTF("BLDC open: duty=%.3f sector=%.3fms dir=%s\r\n", duty, sector_period_ms,
               direction == MOTOR_BLDC_DIRECTION_CCW ? "CCW" : "CW");
        return 0;
    }
    if (!Motor_Bldc_Start(direction, duty)) {
        PRINTF("BLDC start rejected: stop FOC, clear fault, use duty (0,0.25]\r\n");
        return 0;
    }
    PRINTF("BLDC start: duty=%.3f dir=%s\r\n", duty,
           direction == MOTOR_BLDC_DIRECTION_CCW ? "CCW" : "CW");
    return 0;
}

#ifdef __cplusplus
}
#endif
