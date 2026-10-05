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
#define POS_TEST_DWELL_DEFAULT_MS 2000U
#define POS_TEST_DWELL_MIN_MS     500U
#define POS_TEST_DWELL_MAX_MS     10000U
#define POS_TEST_POLL_MS          50U
#define POS_TEST_SETTLE_ERROR_DEG (1.0f)
#define POS_TEST_SETTLE_MS        300U
#define POS_TEST_EXTRA_TIMEOUT_MS 10000U
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if MOTOR_POSITION_SMALL_TEST_SEQUENCE != 0U
static const float s_pos_test_deg[] = {0.0f, 2.0f, 5.0f, -2.0f, 0.0f};
#else
static const float s_pos_test_deg[] = {0.0f, 30.0f, 90.0f, 180.0f, 90.0f, 0.0f, -90.0f, 0.0f};
#endif
static CMD_QUEUE_ITEM s_pos_key;
/* Private function prototypes -----------------------------------------------*/
static void Cmd_PositionPrint(void);
static void pos_emit_buf(const char *buf, uint32_t len);
static void pos_emit_str(const char *text);
static bool pos_stop_key(void);
static bool pos_wait_or_key(uint32_t period_ms);
static int pos_wait_until_settled(void);
static void pos_append_deg(textbuf_t *tb, float deg);
static void pos_test_run(uint32_t dwell_ms);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 把已拼好的字节写入串口发送队列。
 * @param {const char *} buf  内容，可为空
 * @param {uint32_t} len  字节数
 * @return {*} 无
 */
static void pos_emit_buf(const char *buf, uint32_t len)
{
    if ((buf == NULL) || (len == 0U)) {
        return;
    }
    (void)Bsp_Usart_Write((const UCHAR *)buf, (ULONG)len);
}

/**
 * @description: 发送以 0 结尾的字符串。
 * @param {const char *} text  内容，可为空
 * @return {*} 无
 */
static void pos_emit_str(const char *text)
{
    if (text == NULL) {
        return;
    }
    pos_emit_buf(text, (uint32_t)strlen(text));
}

/**
 * @description: 测试过程中是否收到任意按键。
 * @detail: 命令任务正占用在角度序列里，主循环不会取队列。读到一包就停止并清掉后续输入。
 * @return {bool} true=要停止
 */
static bool pos_stop_key(void)
{
    if (queueCmdHandle == NULL) {
        return false;
    }
    if (osMessageQueueGet(queueCmdHandle, &s_pos_key, NULL, 0) != osOK) {
        return false;
    }
    while (osMessageQueueGet(queueCmdHandle, &s_pos_key, NULL, 0) == osOK) {
    }
    return true;
}

/**
 * @description: 等待角度保持时间，中途有按键则提前返回。
 * @param {uint32_t} period_ms  等待时间，单位 ms
 * @return {bool} true=用户按键
 */
static bool pos_wait_or_key(uint32_t period_ms)
{
    uint32_t waited_ms = 0U;

    while (waited_ms < period_ms) {
        uint32_t step_ms = period_ms - waited_ms;

        if (step_ms > POS_TEST_POLL_MS) {
            step_ms = POS_TEST_POLL_MS;
        }
        osDelay(pdMS_TO_TICKS(step_ms));
        waited_ms += step_ms;
        if (pos_stop_key()) {
            return true;
        }
    }
    return false;
}

/**
 * @description: 等待位置进入误差窗口并连续稳定一段时间。
 * @return {int} 1=到位，0=超时，-1=用户中止
 */
static int pos_wait_until_settled(void)
{
    uint32_t waited_ms = 0U;
    uint32_t settled_ms = 0U;

    while (waited_ms < POS_TEST_EXTRA_TIMEOUT_MS) {
        motor_position_state_t state;

        osDelay(pdMS_TO_TICKS(POS_TEST_POLL_MS));
        waited_ms += POS_TEST_POLL_MS;
        if (pos_stop_key()) return -1;
        Motor_Position_GetState(&state);
        if (fabsf(state.error_deg) <= POS_TEST_SETTLE_ERROR_DEG) {
            settled_ms += POS_TEST_POLL_MS;
            if (settled_ms >= POS_TEST_SETTLE_MS) return 1;
        } else {
            settled_ms = 0U;
        }
    }
    return 0;
}

/**
 * @description: 按固定两位小数追加角度。
 * @param {textbuf_t *} tb  输出缓冲
 * @param {float} deg  角度，单位 deg
 * @return {*} 无
 */
static void pos_append_deg(textbuf_t *tb, float deg)
{
    float scaled = deg * 100.0f;
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
 * @detail: 每段先等待指定时间，再确认连续到位；未到位则最多追加等待并安全停机，
 *          禁止在前一目标仍有较大误差时继续切换。角度相对 pos zero。
 * @param {uint32_t} dwell_ms  每个角度的保持时间，单位 ms
 * @return {*} 无
 */
static void pos_test_run(uint32_t dwell_ms)
{
    char line[96];
    textbuf_t tb;
    unsigned int count = sizeof(s_pos_test_deg) / sizeof(s_pos_test_deg[0]);
    unsigned int i;

    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "pos test ");
    (void)textbuf_append_u32(&tb, dwell_ms);
    (void)textbuf_append_str(&tb, " ms, any key stops\r\n");
    pos_emit_buf(line, textbuf_length(&tb));

    for (i = 0U; i < count; i++) {
        motor_position_state_t state;
        int settle_result;

        if (!Motor_Position_SetTargetDeg(s_pos_test_deg[i])) {
            pos_emit_str("pos test: bad target\r\n");
            return;
        }
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_u32(&tb, (uint32_t)(i + 1U));
        (void)textbuf_append_char(&tb, '/');
        (void)textbuf_append_u32(&tb, (uint32_t)count);
        (void)textbuf_append_str(&tb, " target=");
        pos_append_deg(&tb, s_pos_test_deg[i]);
        (void)textbuf_append_str(&tb, " deg\r\n");
        pos_emit_buf(line, textbuf_length(&tb));

        if (pos_wait_or_key(dwell_ms)) {
            pos_emit_str("pos test stopped\r\n");
            return;
        }

        settle_result = pos_wait_until_settled();
        if (settle_result < 0) {
            pos_emit_str("pos test stopped\r\n");
            return;
        }
        if (settle_result == 0) {
            (void)Motor_Position_Enable(false);
            (void)Motor_RequestStop(&motor);
            pos_emit_str("pos test: target not settled, motor stopped\r\n");
            return;
        }

        Motor_Position_GetState(&state);
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_str(&tb, "  pos=");
        pos_append_deg(&tb, state.position_deg);
        (void)textbuf_append_str(&tb, " err=");
        pos_append_deg(&tb, state.error_deg);
        (void)textbuf_append_str(&tb, " vq=");
        pos_append_deg(&tb, motor.position_hold_v);
        (void)textbuf_append_str(&tb, " V\r\n");
        pos_emit_buf(line, textbuf_length(&tb));
    }
    (void)Motor_Position_Enable(false);
    (void)Motor_RequestStop(&motor);
    pos_emit_str("pos test done, motor auto-stopped\r\n");
}

/**
 * @description: 打印位置环使能、角度、速度命令、保持电压和比例限幅。
 * @return {void} 无
 * @author: lkc
 */
static void Cmd_PositionPrint(void)
{
    motor_position_state_t state;
    Motor_Position_GetState(&state);
    PRINTF("Position: enabled=%d mode=%s pos=%.2fdeg target=%.2fdeg err=%.2fdeg vel=%.1fRPM vq=%.3fV kp=%.3f max=%.1fRPM\r\n",
           state.enabled ? 1 : 0, state.voltage_hold_mode ? "HOLD" : "MOVE", state.position_deg,
           state.target_deg, state.error_deg, state.target_rpm, motor.position_hold_v,
           state.kp_rpm_per_deg, state.max_rpm);
}

/**
 * @description: 位置环命令 pos。
 * @detail: zero 只能在停机时改零点。hold/set/move/test/on 要求反馈源为编码器，且首次使能前电机已停。
 *          成功后把模式设为 I/F 并打开速度环；电机尚未运行时仍须 olstart 才上电。
 *          test 在已上电后按预设角度切换，任意键停止。
 * @param {int} argc       参数个数，含命令名
 * @param {char *[]} argv  参数表；子命令在 argv[1]
 * @return {int} 0=已执行，-1=参数或前置条件不满足
 * @author: lkc
 */
int Cmd_Position(int argc, char *argv[])
{
    motor_command_t cmd;
    bool was_enabled = Motor_Position_IsEnabled();
    bool run_test = false;
    uint32_t dwell_ms = POS_TEST_DWELL_DEFAULT_MS;

    if (argc < 2 || (strcmp(argv[1], "status") == 0 && argc == 2)) {
        Cmd_PositionPrint();
        PRINTF("usage: pos zero|hold|set <deg>|move <deg>|test [ms]|on|off|kp <rpm/deg>|max <rpm>\r\n");
        return 0;
    }
    if (strcmp(argv[1], "zero") == 0 && argc == 2) {
        if (motor.ol_enable) {
            PRINTF("Stop motor before changing position zero\r\n");
            return -1;
        }
        Motor_Position_Zero();
        PRINTF("Position zero set at current encoder position\r\n");
        return 0;
    }
    if (strcmp(argv[1], "off") == 0 && argc == 2) {
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
    if (strcmp(argv[1], "hold") == 0 && argc == 2) {
        motor_position_state_t state;
        (void)Motor_Joint_Enable(false);
        Motor_Position_GetState(&state);
        if (!Motor_Position_SetTargetDeg(state.position_deg) || !Motor_Position_Enable(true)) {
            PRINTF("Position loop needs valid calibrated encoder\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "set") == 0 && argc == 3) {
        (void)Motor_Joint_Enable(false);
        if (!Motor_Position_SetTargetDeg((float)atof(argv[2])) || !Motor_Position_Enable(true)) {
            PRINTF("Invalid target or encoder not ready\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "move") == 0 && argc == 3) {
        (void)Motor_Joint_Enable(false);
        if (!Motor_Position_MoveDeg((float)atof(argv[2])) || !Motor_Position_Enable(true)) {
            PRINTF("Invalid move or encoder not ready\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "test") == 0 && argc <= 3) {
        if (argc == 3) {
            dwell_ms = (uint32_t)atoi(argv[2]);
        }
        if (dwell_ms < POS_TEST_DWELL_MIN_MS) {
            dwell_ms = POS_TEST_DWELL_MIN_MS;
        }
        if (dwell_ms > POS_TEST_DWELL_MAX_MS) {
            dwell_ms = POS_TEST_DWELL_MAX_MS;
        }
        (void)Motor_Joint_Enable(false);
        if (!Motor_Position_SetTargetDeg(s_pos_test_deg[0]) || !Motor_Position_Enable(true)) {
            PRINTF("Position loop needs valid calibrated encoder\r\n");
            return -1;
        }
        run_test = true;
    } else if (strcmp(argv[1], "on") == 0 && argc == 2) {
        (void)Motor_Joint_Enable(false);
        if (!Motor_Position_Enable(true)) {
            PRINTF("Position loop needs valid calibrated encoder\r\n");
            return -1;
        }
    } else if (strcmp(argv[1], "kp") == 0 && argc == 3) {
        if (!Motor_Position_SetKp((float)atof(argv[2]))) return -1;
        Cmd_PositionPrint();
        return 0;
    } else if (strcmp(argv[1], "max") == 0 && argc == 3) {
        if (!Motor_Position_SetMaxRpm((float)atof(argv[2]))) {
            PRINTF("Invalid position max RPM for selected motor\r\n");
            return -1;
        }
        Cmd_PositionPrint();
        return 0;
    } else {
        PRINTF("usage: pos zero|hold|set <deg>|move <deg>|test [ms]|on|off|kp <rpm/deg>|max <rpm>\r\n");
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
        return 0;
    }
    if (run_test) {
        pos_test_run(dwell_ms);
    }
    return 0;
}
#ifdef __cplusplus
}
#endif
