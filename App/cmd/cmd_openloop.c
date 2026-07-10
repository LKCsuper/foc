/*
 * @Description: 开环 V/F 控制命令 — 运行时调整转速和 V/F 参数
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-08-30 10:31:14
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-24 23:29:19
 * @detail:
 *   olspeed <rpm>     设置目标转速，负数为反转
 *   olspeed            查看当前开环状态
 *   olboost <V>        设置 V/F 启动补偿电压
 *   olkvf <V/Hz>       设置 V/F 斜率
 *   olramp <RPM/s>     设置软启动斜坡速率
 *   olvlimit <V>       设置 Vq 上限
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    OPEN_LOOP_NONE,
    OPEN_LOOP_CREATE,
    OPEN_LOOP_DESTORY,
    OPEN_LOOP_SET,
    OPEN_LOOP_START,
    OPEN_LOOP_STOP,
    OPEN_LOOP_NUM,
}OPEN_LOOP_TYPE;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
STATIC TaskHandle_t taskPlotHandle;

STATIC const osThreadAttr_t taskDutyAttr =
{
    .name = "Open Loop",
    .priority = PRI_INIT,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 打印开环当前状态
 */
static void ol_print_status(void)
{
    PRINTF("\n===== Open-Loop Status =====\n");
    PRINTF("  Enable     : %s\n", motor.ol_enable ? "ON" : "OFF");
    PRINTF("  Mode       : %d (%s)\n", motor.ol_mode,
           motor.ol_mode == MOTOR_OL_MODE_IF ? "I/F current-loop" : "V/F voltage");
    PRINTF("  Target RPM : %.0f\n", motor.ol_target_rpm);
    PRINTF("  Max RPM    : %.0f\n", motor.ol_max_rpm);
    PRINTF("  Current RPM: %.0f\n", motor.ol_current_rpm);
    PRINTF("  Min RPM    : %.0f\n", motor.ol_min_rpm);
    PRINTF("  Ramp       : %.0f RPM/s\n", motor.ol_ramp_rpm_s);
    PRINTF("  V_bus      : %.1f V\n", motor.v_bus);
    PRINTF("  Fault      : %d (%s)\n", Motor_Protect_GetFault(),
           Motor_Protect_IsFaultLatched() ? "LATCHED" : "NONE");

    if (motor.ol_mode == MOTOR_OL_MODE_IF)
    {
        /* 电流模式 */
        PRINTF("  Iq set     : %.2f A\n", motor.ol_iq_set);
        PRINTF("  Id set     : %.2f A\n", motor.ol_id_set);
        PRINTF("  Current Kp : %.4f\n", motor.ol_current_kp);
        PRINTF("  Current Ki : %.2f\n", motor.ol_current_ki);
        PRINTF("  Vq limit   : %.1f V\n", motor.ol_vq_limit);
        PRINTF("  Start Phs  : %.2f rad\n", motor.ol_start_phase_boost);
        PRINTF("  Ia,Ib,Ic   : %.3f %.3f %.3f A\n",
               motor.ia, motor.ib, motor.ic);
        PRINTF("  Id, Iq     : %.3f %.3f A\n", motor.Id, motor.Iq);
        PRINTF("  Vd, Vq int : %.3f %.3f\n", motor.ol_vd_int, motor.ol_vq_int);
    }
    else
    {
        /* V/F 模式 */
        PRINTF("  V_boost    : %.2f V\n", motor.ol_v_boost);
        PRINTF("  Kv_f       : %.4f V/Hz\n", motor.ol_kv_f);
        PRINTF("  Vq limit   : %.1f V\n", motor.ol_vq_limit);

        float f_elec = fabsf(motor.ol_current_rpm) / 60.0f * motor.ol_pole_pairs;
        float vq_est = motor.ol_v_boost + motor.ol_kv_f * f_elec;
        if (vq_est > motor.ol_vq_limit) vq_est = motor.ol_vq_limit;
        float mod = 0.0f;
        if (motor.v_bus > 1.0f) mod = vq_est / motor.v_bus * 100.0f;
        PRINTF("  Est Vq     : %.2f V (%.0f%% mod)\n", vq_est, mod);
    }
    PRINTF("=============================\n\n");
}

/**
 * @description: olspeed 命令 — 设置/查看开环目标转速
 * @param argc 参数个数
 * @param argv 参数列表
 */
int Cmd_OlSpeed(int argc, char *argv[])
{
    if (argc < 2)
    {
        ol_print_status();
        PRINTF("Usage: olspeed <rpm>  — set target speed, negative for reverse\n");
        return 0;
    }

    float rpm = atof(argv[1]);

    if (motor.ol_max_rpm > 0.0f) {
        if (rpm > motor.ol_max_rpm) {
            rpm = motor.ol_max_rpm;
        } else if (rpm < -motor.ol_max_rpm) {
            rpm = -motor.ol_max_rpm;
        }
    }

    motor.ol_target_rpm = rpm;
    motor.ol_enable = (rpm != 0.0f);
    if (!motor.ol_enable) motor.ol_current_rpm = 0.0f;
    if (motor.ol_enable) Bsp_Tim_StartPwm(MOTOR1);
    PRINTF("Target RPM set to %.0f\n", rpm);
    return 0;
}

/**
 * @description: olboost 命令 — 设置 V/F 启动补偿电压
 */
int Cmd_OlBoost(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("V_boost = %.2f V  (usage: olboost <volts>)\n", motor.ol_v_boost);
        return 0;
    }
    motor.ol_v_boost = atof(argv[1]);
    if (motor.ol_v_boost < 0.0f) motor.ol_v_boost = 0.0f;
    PRINTF("V_boost set to %.2f V\n", motor.ol_v_boost);
    return 0;
}

/**
 * @description: olkvf 命令 — 设置 V/F 斜率
 */
int Cmd_OlKvf(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Kv_f = %.4f V/Hz  (usage: olkvf <V/Hz>)\n", motor.ol_kv_f);
        return 0;
    }
    motor.ol_kv_f = atof(argv[1]);
    if (motor.ol_kv_f < 0.0f) motor.ol_kv_f = 0.0f;
    PRINTF("Kv_f set to %.4f V/Hz\n", motor.ol_kv_f);
    return 0;
}

/**
 * @description: olramp 命令 — 设置软启动斜坡速率
 */
int Cmd_OlRamp(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Ramp = %.0f RPM/s  (usage: olramp <rpm/s>)\n", motor.ol_ramp_rpm_s);
        return 0;
    }
    motor.ol_ramp_rpm_s = atof(argv[1]);
    if (motor.ol_ramp_rpm_s < 0.0f) motor.ol_ramp_rpm_s = 0.0f;
    PRINTF("Ramp set to %.0f RPM/s\n", motor.ol_ramp_rpm_s);
    return 0;
}

/**
 * @description: olvlimit 命令 — 设置 Vq 上限
 */
int Cmd_OlVlimit(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Vq_limit = %.1f V  (usage: olvlimit <volts>)\n", motor.ol_vq_limit);
        return 0;
    }
    motor.ol_vq_limit = atof(argv[1]);
    if (motor.ol_vq_limit < 0.0f) motor.ol_vq_limit = 0.0f;
    PRINTF("Vq_limit set to %.1f V\n", motor.ol_vq_limit);
    return 0;
}

int Cmd_OlMaxSpeed(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Max RPM = %.0f  (usage: olmax <rpm>)\n", motor.ol_max_rpm);
        return 0;
    }

    motor.ol_max_rpm = atof(argv[1]);
    if (motor.ol_max_rpm < 1.0f) motor.ol_max_rpm = 1.0f;
    if (motor.ol_target_rpm > motor.ol_max_rpm) motor.ol_target_rpm = motor.ol_max_rpm;
    if (motor.ol_target_rpm < -motor.ol_max_rpm) motor.ol_target_rpm = -motor.ol_max_rpm;
    PRINTF("Max RPM set to %.0f\n", motor.ol_max_rpm);
    return 0;
}

int Cmd_OlMinSpeed(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Min RPM = %.0f  (usage: olmin <rpm>)\n", motor.ol_min_rpm);
        return 0;
    }

    motor.ol_min_rpm = atof(argv[1]);
    if (motor.ol_min_rpm < 0.0f) motor.ol_min_rpm = 0.0f;
    if (motor.ol_max_rpm > 0.0f && motor.ol_min_rpm > motor.ol_max_rpm)
    {
        motor.ol_min_rpm = motor.ol_max_rpm;
    }
    PRINTF("Min RPM set to %.0f\n", motor.ol_min_rpm);
    return 0;
}

int Cmd_OlStart(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (motor.ol_target_rpm == 0.0f)
    {
        motor.ol_target_rpm = motor.ol_min_rpm;
    }
    if (motor.ol_max_rpm > 0.0f && motor.ol_target_rpm > motor.ol_max_rpm)
    {
        motor.ol_target_rpm = motor.ol_max_rpm;
    }
    else if (motor.ol_max_rpm > 0.0f && motor.ol_target_rpm < -motor.ol_max_rpm)
    {
        motor.ol_target_rpm = -motor.ol_max_rpm;
    }

    motor.ol_enable = true;
    Bsp_Tim_StartPwm(MOTOR1);
    PRINTF("Open-loop started, target %.0f RPM\n", motor.ol_target_rpm);
    return 0;
}

int Cmd_OlStop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    motor.ol_enable = false;
    motor.ol_target_rpm = 0.0f;
    motor.ol_current_rpm = 0.0f;
    motor.ol_vd_int = 0.0f;
    motor.ol_vq_int = 0.0f;
    TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    PRINTF("Open-loop stopped\n");
    return 0;
}

int Cmd_OlFault(int argc, char *argv[])
{
    /* 不带 clear 参数时只查询，避免误操作解除保护。 */
    if (argc < 2 || strcmp(argv[1], "clear") != 0)
    {
        PRINTF("Fault = %d (%s)\n", Motor_Protect_GetFault(),
               Motor_Protect_IsFaultLatched() ? "LATCHED" : "NONE");
        PRINTF("Usage: olfault clear\n");
        return 0;
    }

    /* 先撤销运行命令并清空 PI，防止解除锁存后立即带载重启。 */
    motor.ol_enable = false;
    motor.ol_target_rpm = 0.0f;
    motor.ol_current_rpm = 0.0f;
    motor.ol_vd_int = 0.0f;
    motor.ol_vq_int = 0.0f;
    TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    Motor_Protect_ClearFault();
    PRINTF("Motor fault cleared; use olstart to restart\n");
    return 0;
}

/**
 * @description: olmode 命令 — 切换开环模式
 * @usage: olmode <0|1>  (0=V/F, 1=I/F)
 */
int Cmd_OlMode(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Open-loop mode = %d (%s)\n", motor.ol_mode,
               motor.ol_mode == MOTOR_OL_MODE_IF ? "I/F current-loop" : "V/F voltage");
        PRINTF("Usage: olmode <0|1>\n");
        PRINTF("  0 = V/F voltage mode\n");
        PRINTF("  1 = I/F current closed-loop\n");
        return 0;
    }

    int mode = atoi(argv[1]);
    if (mode != MOTOR_OL_MODE_VF && mode != MOTOR_OL_MODE_IF)
    {
        PRINTF("Invalid mode, use 0 or 1\n");
        return -1;
    }

    motor.ol_mode = mode;
    motor.ol_vd_int = 0.0f;
    motor.ol_vq_int = 0.0f;
    PRINTF("Open-loop mode set to %d (%s)\n", mode,
           mode == 1 ? "Current-loop" : "V/F voltage");
    return 0;
}

/**
 * @description: oliq 命令 — 设置开环电流模式 Iq 目标
 */
int Cmd_OlIq(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Iq set = %.2f A  (usage: oliq <amps>)\n", motor.ol_iq_set);
        return 0;
    }
    motor.ol_iq_set = atof(argv[1]);
    if (motor.ol_iq_set < 0.0f) motor.ol_iq_set = 0.0f;
    PRINTF("Iq set to %.2f A\n", motor.ol_iq_set);
    return 0;
}

/**
 * @description: olid 命令 — 设置开环电流模式 Id 目标（通常为 0）
 */
int Cmd_OlId(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Id set = %.2f A  (usage: olid <amps>)\n", motor.ol_id_set);
        return 0;
    }
    motor.ol_id_set = atof(argv[1]);
    PRINTF("Id set to %.2f A\n", motor.ol_id_set);
    return 0;
}

/**
 * @description: olkp 命令 — 设置开环电流环 Kp
 */
int Cmd_OlKp(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Current Kp = %.4f  (usage: olkp <value>)\n", motor.ol_current_kp);
        return 0;
    }
    motor.ol_current_kp = atof(argv[1]);
    if (motor.ol_current_kp < 0.0f) motor.ol_current_kp = 0.0f;
    PRINTF("Current Kp set to %.4f\n", motor.ol_current_kp);
    return 0;
}

/**
 * @description: olki 命令 — 设置开环电流环 Ki
 */
int Cmd_OlKi(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Current Ki = %.2f  (usage: olki <value>)\n", motor.ol_current_ki);
        return 0;
    }
    motor.ol_current_ki = atof(argv[1]);
    if (motor.ol_current_ki < 0.0f) motor.ol_current_ki = 0.0f;
    PRINTF("Current Ki set to %.2f\n", motor.ol_current_ki);
    return 0;
}

/**
 * @description: olphase 命令 — 设置启动相位补偿
 */
int Cmd_OlPhase(int argc, char *argv[])
{
    if (argc < 2)
    {
        PRINTF("Start phase boost = %.2f rad  (usage: olphase <radians>)\n",
               motor.ol_start_phase_boost);
        return 0;
    }
    motor.ol_start_phase_boost = atof(argv[1]);
    PRINTF("Start phase boost set to %.2f rad\n", motor.ol_start_phase_boost);
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olspeed, Cmd_OlSpeed, openloop V/F speed set);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olboost, Cmd_OlBoost, openloop V/F boost voltage);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olkvf, Cmd_OlKvf, openloop V/F slope);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olramp, Cmd_OlRamp, openloop V/F ramp rate);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olvlimit, Cmd_OlVlimit, openloop Vq limit);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olmax, Cmd_OlMaxSpeed, openloop max speed);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olmin, Cmd_OlMinSpeed, openloop min speed);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olstart, Cmd_OlStart, openloop start);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olstop, Cmd_OlStop, openloop stop);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olfault, Cmd_OlFault, show or clear latched motor fault);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olmode, Cmd_OlMode, openloop mode 0=VF 1=current);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                oliq, Cmd_OlIq, openloop current Iq target);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olid, Cmd_OlId, openloop current Id target);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olkp, Cmd_OlKp, openloop current Kp);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olki, Cmd_OlKi, openloop current Ki);
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                olphase, Cmd_OlPhase, openloop start phase boost);

/**
 * @description: 原 openloop 任务（保留兼容）
 */
VOID Openloop_ControlTask(VOID *argument)
{
    (void)argument;

    PRINTF("\n");
    while (1)
    {
        ol_print_status();
        osDelay(pdMS_TO_TICKS(1000));
    }
}

VOID Cmd_Motor_OpenLoop(int cTaskStat, int cPrecentage)
{
    switch (cTaskStat)
    {
        case OPEN_LOOP_CREATE:
            PRINTF("OpenLoop task create\n");
            taskPlotHandle = osThreadNew(Openloop_ControlTask, NULL, &taskDutyAttr);
            break;
        default:
            break;
    }
    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                openloop, Cmd_Motor_OpenLoop, open loop task);
#ifdef __cplusplus
}
#endif
