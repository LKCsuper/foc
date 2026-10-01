/*
 * @Description: 关节阻抗 Shell 命令，与 pos / olspeed 分开
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
#define JOINT_TEST_DWELL_DEFAULT_MS 2000U
#define JOINT_TEST_DWELL_MIN_MS     500U
#define JOINT_TEST_DWELL_MAX_MS     10000U
#define JOINT_TEST_POLL_MS          50U
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const float s_joint_test_deg[] = {0.0f, 30.0f, 90.0f, 180.0f, 90.0f, 0.0f, -90.0f, 0.0f};
static CMD_QUEUE_ITEM s_joint_key;
/* Private function prototypes -----------------------------------------------*/
static void Cmd_JointPrint(void);
static int Cmd_JointArm(void);
static void joint_emit_buf(const char *buf, uint32_t len);
static void joint_emit_str(const char *text);
static bool joint_stop_key(void);
static bool joint_wait_or_key(uint32_t period_ms);
static void joint_append_fixed(textbuf_t *tb, float value);
static void joint_test_run(uint32_t dwell_ms);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把已拼好的字节写入串口发送队列。
 * @param {const char *} buf  内容，可为空
 * @param {uint32_t} len  字节数
 * @return {void} 无
 * @author: lkc
 */
static void joint_emit_buf(const char *buf, uint32_t len)
{
    if ((buf == NULL) || (len == 0U)) {
        return;
    }
    (void)Bsp_Usart_Write((const UCHAR *)buf, (ULONG)len);
}

/**
 * @description: 发送以 0 结尾的字符串。
 * @param {const char *} text  内容，可为空
 * @return {void} 无
 * @author: lkc
 */
static void joint_emit_str(const char *text)
{
    if (text == NULL) {
        return;
    }
    joint_emit_buf(text, (uint32_t)strlen(text));
}

/**
 * @description: 测试过程中是否收到任意按键。
 * @detail: 命令任务正占用在角度序列里，主循环不会取队列。读到一包就停止并清掉后续输入。
 * @return {bool} true=要停止
 * @author: lkc
 */
static bool joint_stop_key(void)
{
    if (queueCmdHandle == NULL) {
        return false;
    }
    if (osMessageQueueGet(queueCmdHandle, &s_joint_key, NULL, 0) != osOK) {
        return false;
    }
    while (osMessageQueueGet(queueCmdHandle, &s_joint_key, NULL, 0) == osOK) {
    }
    return true;
}

/**
 * @description: 等待角度保持时间，中途有按键则提前返回。
 * @param {uint32_t} period_ms  等待时间，单位 ms
 * @return {bool} true=用户按键
 * @author: lkc
 */
static bool joint_wait_or_key(uint32_t period_ms)
{
    uint32_t waited_ms = 0U;

    while (waited_ms < period_ms) {
        uint32_t step_ms = period_ms - waited_ms;

        if (step_ms > JOINT_TEST_POLL_MS) {
            step_ms = JOINT_TEST_POLL_MS;
        }
        osDelay(pdMS_TO_TICKS(step_ms));
        waited_ms += step_ms;
        if (joint_stop_key()) {
            return true;
        }
    }
    return false;
}

/**
 * @description: 按固定两位小数追加数值。
 * @param {textbuf_t *} tb  输出缓冲
 * @param {float} value  数值
 * @return {void} 无
 * @author: lkc
 */
static void joint_append_fixed(textbuf_t *tb, float value)
{
    float scaled = value * 100.0f;
    int32_t centi;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    centi = (int32_t)scaled;
    (void)textbuf_append_fixed2(tb, centi);
}

/**
 * @description: 依次切到预设角度，任意键停止。
 * @detail: 每段之间 osDelay，不关调度器。转速和力矩前馈保持 0。角度相对 joint zero。
 * @param {uint32_t} dwell_ms  每个角度的保持时间，单位 ms
 * @return {void} 无
 * @author: lkc
 */
static void joint_test_run(uint32_t dwell_ms)
{
    char line[96];
    textbuf_t tb;
    unsigned int count = sizeof(s_joint_test_deg) / sizeof(s_joint_test_deg[0]);
    unsigned int i;

    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "joint test ");
    (void)textbuf_append_u32(&tb, dwell_ms);
    (void)textbuf_append_str(&tb, " ms, any key stops\r\n");
    joint_emit_buf(line, textbuf_length(&tb));

    for (i = 0U; i < count; i++) {
        motor_joint_state_t state;

        if (!Motor_Joint_SetTarget(s_joint_test_deg[i], 0.0f, 0.0f)) {
            joint_emit_str("joint test: bad target\r\n");
            return;
        }
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_u32(&tb, (uint32_t)(i + 1U));
        (void)textbuf_append_char(&tb, '/');
        (void)textbuf_append_u32(&tb, (uint32_t)count);
        (void)textbuf_append_str(&tb, " target=");
        joint_append_fixed(&tb, s_joint_test_deg[i]);
        (void)textbuf_append_str(&tb, " deg\r\n");
        joint_emit_buf(line, textbuf_length(&tb));

        if (joint_wait_or_key(dwell_ms)) {
            joint_emit_str("joint test stopped\r\n");
            return;
        }

        Motor_Joint_GetState(&state);
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_str(&tb, "  pos=");
        joint_append_fixed(&tb, state.position_deg);
        (void)textbuf_append_str(&tb, " err=");
        joint_append_fixed(&tb, state.error_deg);
        (void)textbuf_append_str(&tb, " vel=");
        joint_append_fixed(&tb, state.measured_rpm);
        (void)textbuf_append_str(&tb, " vq=");
        joint_append_fixed(&tb, state.output_v);
        (void)textbuf_append_str(&tb, " bias=");
        joint_append_fixed(&tb, state.bias_v);
        (void)textbuf_append_str(&tb, " V\r\n");
        joint_emit_buf(line, textbuf_length(&tb));
    }
    joint_emit_str("joint test done\r\n");
}

/**
 * @description: 打印关节阻抗使能、角度、速度、力矩前馈和电压。
 * @return {void} 无
 * @author: lkc
 */
static void Cmd_JointPrint(void)
{
    motor_joint_state_t state;

    Motor_Joint_GetState(&state);
    PRINTF("Joint: en=%d pos=%.2f target=%.2f err=%.2f vel=%.1f vdes=%.1f "
           "tff=%.3fA vq=%.3fV kp=%.3f kd=%.3f ki=%.3f bias=%.3f lim=%.2fV\r\n",
           state.enabled ? 1 : 0, state.position_deg, state.target_deg, state.error_deg, state.measured_rpm,
           state.target_rpm, state.torque_ff_a, state.output_v, state.kp_v_per_deg, state.kd_v_per_rpm,
           state.ki_v_per_deg_s, state.bias_v, state.v_limit_v);
}

/**
 * @description: 切到 I/F 并打开速度环，转速命令保持 0，避免和阻抗电压叠加。
 * @return {int} 0=已写入，-1=命令被拒绝
 * @author: lkc
 */
static int Cmd_JointArm(void)
{
    motor_command_t cmd;

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
    }
    cmd.mode = MOTOR_OL_MODE_IF;
    cmd.speed_enable = true;
    cmd.target_rpm = 0.0f;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        return -1;
    }
    return 0;
}

/**
 * @description: 关节阻抗命令 joint。
 * @detail: 与 pos、olspeed 互斥。set 的三个数是目标角度 (deg)、目标转速 (RPM)、力矩前馈 (A)。
 *          首次使能前电机须停转。尚未上电时提示 olstart。
 *          test 在已上电后按预设角度切换，任意键停止。
 * @param {int} argc  参数个数
 * @param {char *[]} argv  参数表
 * @return {int} 0=已执行，-1=参数或前置条件不满足
 * @author: lkc
 */
int Cmd_Joint(int argc, char *argv[])
{
    motor_joint_state_t state;
    bool was_enabled = Motor_Joint_IsEnabled();
    float target_deg;
    float target_rpm;
    float torque_ff_a;
    bool tune_only;
    bool run_test = false;
    uint32_t dwell_ms = JOINT_TEST_DWELL_DEFAULT_MS;

    if (argc < 2 || strcmp(argv[1], "status") == 0) {
        Cmd_JointPrint();
        PRINTF("usage: joint zero|on|off|set <deg> [rpm] [A]|test [ms]|kp <V/deg>|kd <V/RPM>|ki <V/deg/s>|lim <V>\r\n");
        return 0;
    }
    if (strcmp(argv[1], "zero") == 0) {
        if (motor.ol_enable) {
            PRINTF("Stop motor before changing joint zero\r\n");
            return -1;
        }
        Motor_Joint_Zero();
        PRINTF("Joint zero set at current encoder position\r\n");
        return 0;
    }
    if (strcmp(argv[1], "off") == 0) {
        (void)Motor_Joint_Enable(false);
        PRINTF("Joint impedance disabled\r\n");
        return 0;
    }
    if (Motor_GetFeedbackSource() != MOTOR_FEEDBACK_SRC_ENCODER) {
        PRINTF("Joint impedance requires: olfb encoder\r\n");
        return -1;
    }
    tune_only = strcmp(argv[1], "kp") == 0 || strcmp(argv[1], "kd") == 0;
    tune_only = tune_only || strcmp(argv[1], "ki") == 0 || strcmp(argv[1], "lim") == 0;
    if (!was_enabled && motor.ol_enable && !tune_only) {
        PRINTF("Stop motor before enabling joint impedance\r\n");
        return -1;
    }
    Motor_Joint_GetState(&state);
    if (strcmp(argv[1], "on") == 0) {
        (void)Motor_Position_Enable(false);
        if (!Motor_Joint_SetTarget(state.position_deg, 0.0f, 0.0f) || !Motor_Joint_Enable(true)) {
            PRINTF("Joint impedance needs valid calibrated encoder\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "set") == 0 && argc >= 3) {
        target_deg = (float)atof(argv[2]);
        target_rpm = (argc >= 4) ? (float)atof(argv[3]) : state.target_rpm;
        torque_ff_a = (argc >= 5) ? (float)atof(argv[4]) : state.torque_ff_a;
        (void)Motor_Position_Enable(false);
        if (!Motor_Joint_SetTarget(target_deg, target_rpm, torque_ff_a) || !Motor_Joint_Enable(true)) {
            PRINTF("Invalid joint target or encoder not ready\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "test") == 0) {
        if (argc >= 3) {
            dwell_ms = (uint32_t)atoi(argv[2]);
        }
        if (dwell_ms < JOINT_TEST_DWELL_MIN_MS) {
            dwell_ms = JOINT_TEST_DWELL_MIN_MS;
        }
        if (dwell_ms > JOINT_TEST_DWELL_MAX_MS) {
            dwell_ms = JOINT_TEST_DWELL_MAX_MS;
        }
        (void)Motor_Position_Enable(false);
        if (!Motor_Joint_SetTarget(s_joint_test_deg[0], 0.0f, 0.0f) || !Motor_Joint_Enable(true)) {
            PRINTF("Joint impedance needs valid calibrated encoder\r\n");
            return -1;
        }
        run_test = true;
    } else if (strcmp(argv[1], "kp") == 0 && argc >= 3) {
        if (!Motor_Joint_SetKp((float)atof(argv[2]))) {
            return -1;
        }
        Cmd_JointPrint();
        return 0;
    } else if (strcmp(argv[1], "kd") == 0 && argc >= 3) {
        if (!Motor_Joint_SetKd((float)atof(argv[2]))) {
            return -1;
        }
        Cmd_JointPrint();
        return 0;
    } else if (strcmp(argv[1], "ki") == 0 && argc >= 3) {
        if (!Motor_Joint_SetKi((float)atof(argv[2]))) {
            return -1;
        }
        Cmd_JointPrint();
        return 0;
    } else if (strcmp(argv[1], "lim") == 0 && argc >= 3) {
        if (!Motor_Joint_SetVoltageLimit((float)atof(argv[2]))) {
            return -1;
        }
        Cmd_JointPrint();
        return 0;
    } else {
        PRINTF("usage: joint zero|on|off|set <deg> [rpm] [A]|test [ms]|kp <V/deg>|kd <V/RPM>|ki <V/deg/s>|lim <V>\r\n");
        return -1;
    }
    if (Cmd_JointArm() != 0) {
        return -1;
    }
    Cmd_JointPrint();
    if (!motor.ol_enable) {
        PRINTF("Use olstart to energize joint impedance\r\n");
        return 0;
    }
    if (run_test) {
        joint_test_run(dwell_ms);
    }
    return 0;
}

#ifdef __cplusplus
}
#endif
