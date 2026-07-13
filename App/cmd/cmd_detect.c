/*
 * @Description: 
 * @Version: 2.0
 * @Date: 2023-03-21 23:47:18
 * @LastEditors: lkc
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum tagDetectType
{
    DETECT_NONE,
    DETECT_OFFSET,
    DETECT_HALL,
    DETECT_MOTOR,
    DETECT_SHOW,
    DETECT_APPLY,
    DETECT_POLES,
    DETECT_ABORT,
    DETECT_NUM,
}DETECT_TYPE;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef struct {
    float current_a;
    float electrical_rpm;
} detect_motor_task_args_t;

static detect_motor_task_args_t s_detect_motor_args;
static volatile bool s_detect_motor_task_running = false;
static const osThreadAttr_t s_detect_motor_task_attr = {
    .name = "Motor Detect",
    .priority = PRI_INIT,
    .stack_size = 4096U,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void Detect_PrintUsage(void)
{
    PRINTF("Usage:\n");
    PRINTF("  detect | detect offset | detect 1\n");
    PRINTF("  detect hall | detect 2\n");
    PRINTF("  detect motor [current_A] [electrical_rpm]\n");
    PRINTF("  detect abort  (olstop also aborts motor detection)\n");
    PRINTF("  detect show\n");
    PRINTF("  detect apply\n");
    PRINTF("  detect poles <electrical_rpm> <measured_mechanical_rpm>\n");
}

static void Detect_PrintResult(const motor_detect_result_t *result)
{
    PRINTF("\n===== Motor Detect Result =====\n");
    PRINTF("  Status     : %s\n", DETECT_GetErrorName(result->error));
    PRINTF("  Fault      : %d %s\n", result->fault,
           Motor_Protect_GetFaultName(result->fault));
    PRINTF("  Resistance : %s %.6f ohm\n",
           result->resistance_valid ? "VALID" : "INVALID",
           result->resistance_ohm);
    PRINTF("  Inductance : %s %.3f uH\n",
           result->inductance_valid ? "VALID" : "INVALID",
           result->inductance_h * 1000000.0f);
    PRINTF("  Flux       : %s %.4f mWb\n",
           result->flux_valid ? "VALID" : "INVALID",
           result->flux_linkage_wb * 1000.0f);
    PRINTF("  Pole pairs : %s %.0f\n",
           result->pole_pairs_valid ? "VALID" : "NEEDS MECHANICAL REFERENCE",
           result->pole_pairs);
    PRINTF("  Test       : %.2f A, %.0f electrical RPM\n",
           result->test_current_a, result->test_electrical_rpm);
    if (result->resistance_valid && result->inductance_valid) {
        /* 与 VESC 检测流程相同，按 1.5ms 电流环时间常数给出建议值，但不自动应用。 */
        PRINTF("  PI suggest : Kp=%.4f Ki=%.2f (1.5ms time constant)\n",
               result->inductance_h / 0.0015f,
               result->resistance_ohm / 0.0015f);
    }
    PRINTF("================================\n");
}

static void Detect_MotorTask(void *argument)
{
    motor_detect_result_t result;
    bool success;

    (void)argument;
    success = DETECT_MotorParameters(s_detect_motor_args.current_a,
                                     s_detect_motor_args.electrical_rpm,
                                     &result);
    Detect_PrintResult(&result);
    if (success) {
        PRINTF("Review values, then run: detect apply\n");
        PRINTF("Pole pairs still require tachometer/Hall/encoder reference\n");
    }
    PRINTF("Detect motor finished: %s\n", success ? "SUCCESS" : "FAILED");
    s_detect_motor_task_running = false;
    osThreadExit();
}

static DETECT_TYPE Detect_ParseType(int argc, char *argv[])
{
    if (argc < 2) {
        return DETECT_OFFSET;
    }
    if (strcmp(argv[1], "1") == 0 || strcmp(argv[1], "offset") == 0) {
        return DETECT_OFFSET;
    }
    if (strcmp(argv[1], "2") == 0 || strcmp(argv[1], "hall") == 0) {
        return DETECT_HALL;
    }
    if (strcmp(argv[1], "motor") == 0) {
        return DETECT_MOTOR;
    }
    if (strcmp(argv[1], "show") == 0) {
        return DETECT_SHOW;
    }
    if (strcmp(argv[1], "apply") == 0) {
        return DETECT_APPLY;
    }
    if (strcmp(argv[1], "poles") == 0) {
        return DETECT_POLES;
    }
    if (strcmp(argv[1], "abort") == 0) {
        return DETECT_ABORT;
    }
    return DETECT_NONE;
}

int Cmd_Detect(int argc, char *argv[])
{
    DETECT_TYPE detect_type = Detect_ParseType(argc, argv);
    motor_detect_result_t result;
    bool success = false;

    /*
     * 参数检测会直接驱动功率桥。开始前必须卸载电机负载、固定控制器、
     * 确保电源限流合适，并先处理已有故障。检测结束后始终保持停机。
     */
    if (detect_type == DETECT_NONE) {
        Detect_PrintUsage();
        return -1;
    }

    PRINTF("\n==================================================\n");
    PRINTF("Detect Start\n");

    switch (detect_type)
    {
        case DETECT_OFFSET:
            DETECT_ClearAbortRequest();
            success = DETECT_Offset(FALSE) == 1U;
            break;
        case DETECT_HALL:
            DETECT_Hall();
            success = true;
            break;
        case DETECT_MOTOR: {
            float current_a = argc >= 3 ? (float)atof(argv[2]) : 0.8f;
            float electrical_rpm = argc >= 4 ? (float)atof(argv[3]) : 1200.0f;
            TaskHandle_t task;

            if (s_detect_motor_task_running || Motor_Detect_IsActive()) {
                PRINTF("Motor detection is already running\n");
                success = false;
                break;
            }
            s_detect_motor_args.current_a = current_a;
            s_detect_motor_args.electrical_rpm = electrical_rpm;
            DETECT_ClearAbortRequest();
            s_detect_motor_task_running = true;
            task = osThreadNew(Detect_MotorTask, NULL, &s_detect_motor_task_attr);
            if (task == NULL) {
                s_detect_motor_task_running = false;
                PRINTF("Failed to create motor detection task\n");
                success = false;
                break;
            }
            PRINTF("WARNING: motor will align, pulse, and spin without load\n");
            PRINTF("Background detection started; use olstop or detect abort to stop\n");
            success = true;
            break;
        }
        case DETECT_SHOW:
            DETECT_GetResult(&result);
            Detect_PrintResult(&result);
            success = true;
            break;
        case DETECT_APPLY:
            success = DETECT_ApplyResult(true);
            PRINTF(success ?
                   "Detected R/L/flux applied and saved\n" :
                   "No complete valid result, or motor is running\n");
            break;
        case DETECT_POLES: {
            float pole_pairs;
            float electrical_rpm;
            float mechanical_rpm;

            if (argc < 4) {
                Detect_PrintUsage();
                success = false;
                break;
            }
            electrical_rpm = (float)atof(argv[2]);
            mechanical_rpm = (float)atof(argv[3]);
            success = DETECT_CalculatePolePairs(electrical_rpm, mechanical_rpm,
                                                &pole_pairs);
            if (success) {
                PRINTF("Pole pairs = %.0f (electrical %.1f / mechanical %.1f)\n",
                       pole_pairs, electrical_rpm, mechanical_rpm);
                PRINTF("Run detect apply to save it with the motor result\n");
            } else {
                PRINTF("Pole-pair calculation failed: mechanical reference invalid\n");
            }
            break;
        }
        case DETECT_ABORT:
            success = s_detect_motor_task_running || Motor_Detect_IsActive();
            DETECT_Abort();
            PRINTF(success ? "Detection abort requested\n" :
                             "No active motor detection\n");
            break;
        default:
            break;
    }

    if (detect_type == DETECT_MOTOR && success) {
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
