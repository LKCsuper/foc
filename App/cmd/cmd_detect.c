/*
 * @Description: 偏移校准与霍尔检测命令
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include <math.h>
#include <stdlib.h>
#include "motor_detect_offset.h"
#include "motor_crit.h"
#include "motor_bldc.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum tagDetectType {
    DETECT_NONE,
    DETECT_OFFSET,
    DETECT_HALL,
    DETECT_PHASE,
    DETECT_RL,
    DETECT_MOTOR,
    DETECT_FLUX_SCAN,
    DETECT_SHOW,
    DETECT_ABORT,
} DETECT_TYPE;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static float s_detect_motor_current_a;
static float s_detect_motor_erpm;
static float s_detect_flux_current_a;
static bool s_detect_full_parameters;
static bool s_detect_flux_scan;
static volatile bool s_detect_motor_task_running;
static const osThreadAttr_t s_detect_motor_task_attr = {
    .name = "Motor Detect",
    .priority = PRI_INIT,
    .stack_size = 3072U,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 打印 detect 命令用法
 * @return {*}
 * @author: lkc
 */
static void Detect_PrintUsage(void)
{
    PRINTF("Usage:\n");
    PRINTF("  detect | detect offset [ram] | detect 1\n");
    PRINTF("  detect offset ram set ADC0 ADC1 ADC2 (stopped, +/-1 count, RAM only)\n");
    PRINTF("  detect hall | detect 2\n");
    PRINTF("  detect phase | detect 3   — UVW order vs hall steps\n");
    PRINTF("  detect rl [test_current_A]\n");
    PRINTF("  detect motor [rl_current_A] [electrical_rpm] [flux_current_A]\n");
    PRINTF("  detect fluxscan [rl_current_A] [max_eRPM 900..3000] [flux_current_A]\n");
    PRINTF("  detect show\n");
    PRINTF("  detect abort\n");
}

static bool Detect_ParseFloat(const char *text, float *value)
{
    char *end;
    *value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(*value);
}

static void Detect_PrintMotorResult(const motor_detect_result_t *result)
{
    PRINTF("\n===== Motor Parameter Detect =====\n");
    PRINTF("  Status     : %s\n", DETECT_GetErrorName(result->error));
    PRINTF("  Fault      : %d\n", result->fault);
    PRINTF("  Resistance : %s %.6f ohm\n", result->resistance_valid ? "VALID" : "INVALID",
           (double)result->resistance_ohm);
    PRINTF("  R fit      : offset=%.4f V R2=%.4f\n", (double)result->inverter_voltage_offset_v,
           (double)result->resistance_fit_r2);
    PRINTF("  Inductance : %s %.3f uH\n", result->inductance_valid ? "VALID" : "INVALID",
           (double)(result->inductance_h * 1000000.0f));
    if (result->inductance_valid) {
        uint32_t i;
        PRINTF("  Ld / Lq    : %.3f / %.3f uH\n", (double)(result->inductance_d_h * 1000000.0f),
               (double)(result->inductance_q_h * 1000000.0f));
        PRINTF("  L scan     :");
        for (i = 0U; i < MOTOR_DETECT_INDUCTANCE_DIRECTIONS; i++) {
            PRINTF(" %.1f", (double)(result->inductance_direction_h[i] * 1000000.0f));
        }
        PRINTF(" uH\n");
    }
    if (s_detect_full_parameters) {
        PRINTF("  Flux       : %s %.4f mWb\n", result->flux_valid ? "VALID" : "INVALID",
               (double)(result->flux_linkage_wb * 1000.0f));
        PRINTF("  Test speed : %.0f eRPM\n", (double)result->test_electrical_rpm);
        PRINTF("  Flux current: %.2f A\n", (double)result->test_flux_current_a);
    }
    if (s_detect_flux_scan) {
        PRINTF("  Flux fit   : %s %.4f mWb offset=%.4fV R2=%.4f (encoder reference)\n",
               result->flux_scan_valid ? "VALID" : "INVALID", (double)(result->flux_scan_fit_wb * 1000.0f),
               (double)result->flux_scan_offset_v, (double)result->flux_scan_r2);
    }
    PRINTF("  R/L current: %.2f A\n", (double)result->test_current_a);
    PRINTF("  Result is RAM-only; no config or Flash was changed.\n");
    PRINTF("============================\n");
}

static void Detect_MotorTask(void *argument)
{
    motor_detect_result_t result;
    bool success;

    (void)argument;
    osDelay(pdMS_TO_TICKS(10));
    if (s_detect_flux_scan) {
        success = DETECT_MotorFluxScan(s_detect_motor_current_a, s_detect_motor_erpm, s_detect_flux_current_a, &result);
    } else if (s_detect_full_parameters) {
        success = DETECT_MotorParameters(s_detect_motor_current_a, s_detect_motor_erpm, s_detect_flux_current_a,
                                         &result);
    } else {
        success = DETECT_MotorRl(s_detect_motor_current_a, &result);
    }
    Detect_PrintMotorResult(&result);
    PRINTF("Detect motor finished: %s\n", success ? "SUCCESS" : "FAILED");
    s_detect_motor_task_running = false;
    osThreadExit();
}

/**
 * @description: 解析 detect 子命令
 * @param {int} argc
 * @param {char} *argv[]
 * @return {DETECT_TYPE}
 * @author: lkc
 */
static DETECT_TYPE Detect_ParseType(int argc, char *argv[])
{
    if (argc < 2) return DETECT_OFFSET;
    if (strcmp(argv[1], "1") == 0 || strcmp(argv[1], "offset") == 0) return DETECT_OFFSET;
    if (strcmp(argv[1], "2") == 0 || strcmp(argv[1], "hall") == 0) return DETECT_HALL;
    if (strcmp(argv[1], "3") == 0 || strcmp(argv[1], "phase") == 0) return DETECT_PHASE;
    if (strcmp(argv[1], "fluxscan") == 0) return DETECT_FLUX_SCAN;
    if (strcmp(argv[1], "motor") == 0) return DETECT_MOTOR;
    if (strcmp(argv[1], "rl") == 0) return DETECT_RL;
    if (strcmp(argv[1], "show") == 0) return DETECT_SHOW;
    if (strcmp(argv[1], "abort") == 0) return DETECT_ABORT;
    return DETECT_NONE;
}

/**
 * @description: 电流/电压偏移、霍尔表或 UVW 线序检测
 * @param {int} argc
 * @param {char} *argv[]
 * @return {int} 0 成功，-1 失败
 * @author: lkc
 */
int Cmd_Detect(int argc, char *argv[])
{
    DETECT_TYPE detect_type = Detect_ParseType(argc, argv);
    bool success = false;
    motor_phase_order_report_t phase_report;
    motor_detect_result_t motor_result;

    if (detect_type == DETECT_NONE) {
        Detect_PrintUsage();
        return -1;
    }
    if (detect_type != DETECT_ABORT && detect_type != DETECT_SHOW &&
        (s_detect_motor_task_running || Motor_Detect_IsActive())) {
        PRINTF("Motor detection is already running\n");
        return -1;
    }

    PRINTF("\n==================================================\n");
    PRINTF("Detect Start\n");

    switch (detect_type) {
    case DETECT_OFFSET:
        /* 停机诊断：只在 RAM 重放零偏，各相相对现值最多 1 个 ADC count，不写 Flash。 */
        if (argc == 7 && strcmp(argv[2], "ram") == 0 && strcmp(argv[3], "set") == 0) {
            float values[3];
            uint32_t key;
            if (!Detect_ParseFloat(argv[4], &values[0]) || !Detect_ParseFloat(argv[5], &values[1]) ||
                !Detect_ParseFloat(argv[6], &values[2]) || Motor_Bldc_IsSelected()) break;
            key = Motor_Crit_Enter();
            success = Motor_Detect_OffsetSetCurrentRam(&motor, values);
            Motor_Crit_Exit(key);
            PRINTF(success ? "RAM current offsets replayed\n" : "Offset replay rejected: stop and stay within 1 ADC count\n");
            break;
        }
        if (argc > 3 || (argc == 3 && strcmp(argv[2], "ram") != 0)) {
            PRINTF("Invalid offset arguments: detect offset [ram]\n");
            break;
        }
        success = (argc == 3 ? DETECT_OffsetRam(false) : DETECT_Offset(false)) == 1U;
        break;
    case DETECT_HALL:
        success = DETECT_Hall();
        break;
    case DETECT_PHASE:
        success = Motor_PhaseOrder_Detect(&phase_report) == MOTOR_PHASE_ORDER_OK;
        break;
    case DETECT_RL:
    case DETECT_MOTOR:
    case DETECT_FLUX_SCAN: {
        osThreadId_t task;

        if (s_detect_motor_task_running || Motor_Detect_IsActive()) {
            PRINTF("Motor detection is already running\n");
            break;
        }
        s_detect_flux_scan = detect_type == DETECT_FLUX_SCAN;
        s_detect_motor_current_a = s_detect_flux_scan ? 0.5f : 0.8f;
        s_detect_motor_erpm = s_detect_flux_scan ? 3000.0f : 1200.0f;
        s_detect_flux_current_a = s_detect_flux_scan ? 0.3f : 1.5f;
        if (argc > (detect_type == DETECT_RL ? 3 : 5) ||
            (argc >= 3 && !Detect_ParseFloat(argv[2], &s_detect_motor_current_a)) ||
            (argc >= 4 && !Detect_ParseFloat(argv[3], &s_detect_motor_erpm)) ||
            (argc >= 5 && !Detect_ParseFloat(argv[4], &s_detect_flux_current_a))) {
            PRINTF("Invalid detection arguments: finite numbers required\n");
            break;
        }
        s_detect_full_parameters = detect_type == DETECT_MOTOR || s_detect_flux_scan;
        DETECT_ClearAbortRequest();
        PRINTF("WARNING: motor will align and receive bipolar voltage pulses\n");
        PRINTF(s_detect_full_parameters ? "R/L/flux detection starting; use detect abort or olstop to stop\n"
                                        : "R/L detection starting; use detect abort or olstop to stop\n");
        s_detect_motor_task_running = true;
        task = osThreadNew(Detect_MotorTask, NULL, &s_detect_motor_task_attr);
        if (task == NULL) {
            s_detect_motor_task_running = false;
            PRINTF("Failed to create motor detection task\n");
            break;
        }
        success = true;
        break;
    }
    case DETECT_SHOW:
        DETECT_GetResult(&motor_result);
        Detect_PrintMotorResult(&motor_result);
        success = true;
        break;
    case DETECT_ABORT:
        success = s_detect_motor_task_running || Motor_Detect_IsActive();
        DETECT_Abort();
        PRINTF(success ? "Detection abort requested\n" : "No active motor detection\n");
        break;
    default:
        break;
    }

    if ((detect_type == DETECT_MOTOR || detect_type == DETECT_RL) && success) {
        PRINTF("Detect command accepted: background task is running\n");
    } else {
        PRINTF("Detect End: %s\n", success ? "SUCCESS" : "FAILED");
    }
    PRINTF("==================================================\n");

    return success ? 0 : -1;
}

/* Cmd_Detect 注册在 command.c 命令表中 */

#ifdef __cplusplus
}
#endif
