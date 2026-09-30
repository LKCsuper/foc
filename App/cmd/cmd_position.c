/*
 * @Description: 编码器多圈位置环 Shell 命令
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Cmd_PositionPrint(void);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 打印位置环使能、角度、速度命令、保持电压和比例限幅。
 * @return {void} 无
 * @author: lkc
 */
static void Cmd_PositionPrint(void)
{
    motor_position_state_t state;
    Motor_Position_GetState(&state);
    PRINTF("Position: enabled=%d pos=%.2fdeg target=%.2fdeg err=%.2fdeg vel=%.1fRPM vq=%.3fV kp=%.3f max=%.1fRPM\r\n",
           state.enabled ? 1 : 0, state.position_deg, state.target_deg, state.error_deg, state.target_rpm,
           motor.position_hold_v, state.kp_rpm_per_deg, state.max_rpm);
}

/**
 * @description: 位置环命令 pos。
 * @detail: zero 只能在停机时改零点。hold/set/move/on 要求反馈源为编码器，且首次使能前电机已停。
 *          成功后把模式设为 I/F 并打开速度环；电机尚未运行时仍须 olstart 才上电。
 * @param {int} argc       参数个数，含命令名
 * @param {char *[]} argv  参数表；子命令在 argv[1]
 * @return {int} 0=已执行，-1=参数或前置条件不满足
 * @author: lkc
 */
int Cmd_Position(int argc, char *argv[])
{
    motor_command_t cmd;
    bool was_enabled = Motor_Position_IsEnabled();

    if (argc < 2 || strcmp(argv[1], "status") == 0) {
        Cmd_PositionPrint();
        PRINTF("usage: pos zero|hold|set <deg>|move <deg>|on|off|kp <rpm/deg>|max <rpm>\r\n");
        return 0;
    }
    if (strcmp(argv[1], "zero") == 0) {
        if (motor.ol_enable) {
            PRINTF("Stop motor before changing position zero\r\n");
            return -1;
        }
        Motor_Position_Zero();
        PRINTF("Position zero set at current encoder position\r\n");
        return 0;
    }
    if (strcmp(argv[1], "off") == 0) {
        (void)Motor_Position_Enable(false);
        if (Motor_GetCommand(&motor, &cmd) == MOTOR_OK) {
            cmd.target_rpm = 0.0f;
            (void)Motor_SetCommand(&motor, &cmd);
        }
        PRINTF("Position loop disabled\r\n");
        return 0;
    }
    if (Motor_GetFeedbackSource() != MOTOR_FEEDBACK_SRC_ENCODER) {
        PRINTF("Position loop requires: olfb encoder\r\n");
        return -1;
    }
    if (!was_enabled && motor.ol_enable) {
        PRINTF("Stop motor before enabling position loop\r\n");
        return -1;
    }
    if (strcmp(argv[1], "hold") == 0) {
        motor_position_state_t state;
        Motor_Position_GetState(&state);
        if (!Motor_Position_SetTargetDeg(state.position_deg) || !Motor_Position_Enable(true)) {
            PRINTF("Position loop needs valid calibrated encoder\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "set") == 0 && argc >= 3) {
        if (!Motor_Position_SetTargetDeg((float)atof(argv[2])) || !Motor_Position_Enable(true)) {
            PRINTF("Invalid target or encoder not ready\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "move") == 0 && argc >= 3) {
        if (!Motor_Position_MoveDeg((float)atof(argv[2])) || !Motor_Position_Enable(true)) {
            PRINTF("Invalid move or encoder not ready\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "on") == 0) {
        if (!Motor_Position_Enable(true)) {
            PRINTF("Position loop needs valid calibrated encoder\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "kp") == 0 && argc >= 3) {
        if (!Motor_Position_SetKp((float)atof(argv[2]))) return -1;
        Cmd_PositionPrint();
        return 0;
    } else if (strcmp(argv[1], "max") == 0 && argc >= 3) {
        if (!Motor_Position_SetMaxRpm((float)atof(argv[2]))) return -1;
        Cmd_PositionPrint();
        return 0;
    } else {
        PRINTF("usage: pos zero|hold|set <deg>|move <deg>|on|off|kp <rpm/deg>|max <rpm>\r\n");
        return -1;
    }

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    cmd.mode = MOTOR_OL_MODE_IF;
    cmd.speed_enable = true;
    cmd.target_rpm = 0.0f;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    Cmd_PositionPrint();
    if (!motor.ol_enable) {
        PRINTF("Use olstart to energize position control\r\n");
    }
    return 0;
}
#ifdef __cplusplus
}
#endif
