/*
 * @Description: 速度环命令：sp* 配置、速度阶梯测试与速度前馈检测
 * @Author: lkc
 * @detail: 自 cmd_openloop.c 拆出，命令仍由 command.c 命令表注册。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "cmd_openloop.h"
#include "cmd_speed.h"
#include "common_assert.h"
#include "motor_control.h"
#include "motor_foc_isr.h"
#include "motor_timing.h"
#include "motor_speed_ff_detect.h"
#include "motor_crit.h"
#include "motor_output.h"
#include "motor_foc.h"
#include "motor_current.h"
#include "motor_pid.h"
#include "motor_reference_trig.h"
#include "motor_feedback.h"
#include "motor_mt6701.h"
#include "hal_motor.h"
#include <math.h>
#include <stdarg.h>/* Private functions ---------------------------------------------------------*/
#define SPEED_TEST_DWELL_DEFAULT_MS 2000U
#define SPEED_TEST_DWELL_MIN_MS     500U
#define SPEED_TEST_DWELL_MAX_MS     10000U
#define SPEED_TEST_POLL_MS          50U
#define SPEED_FF_SETTLE_MS          2500U
#define SPEED_FF_SAMPLE_COUNT       20U
#define SPEED_FF_SAMPLE_MS          50U

static const float s_speed_test_rpm[] = {
    150.0f, 300.0f, 600.0f, 300.0f, 150.0f, 0.0f, -150.0f, -300.0f, -150.0f, 0.0f,
};

static bool speed_wait_or_key(uint32_t period_ms);
static bool speed_test_set_rpm(float rpm);

static bool speed_ff_set_enable(bool enable)
{
    motor_command_t cmd;
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return false;
    cmd.enable = enable;
    return Motor_SetCommand(&motor, &cmd) == MOTOR_OK;
}

static bool speed_ff_measure_point(float target_rpm, motor_speed_ff_fit_t *fit)
{
    float rpm_sum = 0.0f;
    float vq_sum = 0.0f;
    float measured;
    float allowed_error;
    unsigned i;

    if (!speed_test_set_rpm(target_rpm)) return false;
    if (speed_wait_or_key(SPEED_FF_SETTLE_MS)) return false;
    for (i = 0U; i < SPEED_FF_SAMPLE_COUNT; i++) {
        if (Motor_Protect_IsFaultLatched() || !Motor_SpeedLoop_IsFeedbackValid()) return false;
        rpm_sum += Motor_SpeedLoop_GetMeasuredRpm();
        vq_sum += motor.ol_vq_int;
        if (speed_wait_or_key(SPEED_FF_SAMPLE_MS)) return false;
    }
    measured = rpm_sum / (float)SPEED_FF_SAMPLE_COUNT;
    vq_sum /= (float)SPEED_FF_SAMPLE_COUNT;
    allowed_error = fmaxf(20.0f, fabsf(target_rpm) * 0.05f);
    if (fabsf(target_rpm - measured) > allowed_error || fabsf(vq_sum) > MOTOR_ENCODER_SPEED_VOLT_LIMIT_V * 0.90f ||
        !Motor_SpeedFfFit_Add(fit, measured, vq_sum)) {
        PRINTF("  reject target=%.0f meas=%.1f vq=%.3f\n", target_rpm, measured, vq_sum);
        return false;
    }
    PRINTF("  target=%+.0f meas=%+.1f vq=%+.3f V\n", target_rpm, measured, vq_sum);
    return true;
}

static bool speed_ff_run_direction(float sign, float max_rpm, motor_speed_ff_fit_t *fit)
{
    unsigned i;
    for (i = 1U; i <= 4U; i++) {
        if (!speed_ff_measure_point(sign * max_rpm * (float)i / 4.0f, fit)) return false;
    }
    return true;
}

static int speed_ff_detect(float max_rpm)
{
    motor_speed_ff_fit_t fit;
    motor_speed_ff_result_t result;
    motor_command_t initial_cmd;
    bool initial_braking;
    bool ok = false;

    if (Motor_GetCommand(&motor, &initial_cmd) != MOTOR_OK) return -1;
    initial_braking = motor.speed_allow_braking;
    if (initial_cmd.enable) {
        PRINTF("speed ff detect requires stopped motor: olstop\n");
        return -1;
    }
    if (Motor_Protect_IsFaultLatched() || Motor_Feedback_GetSourceDefault() != MOTOR_FEEDBACK_SRC_ENCODER ||
        !Motor_Encoder_IsZeroValid()) {
        PRINTF("speed ff detect needs clear fault and calibrated encoder feedback\n");
        return -1;
    }
    if (max_rpm < 600.0f || max_rpm > motor.ol_max_rpm) {
        PRINTF("speed ff max RPM must be 600..%.0f\n", motor.ol_max_rpm);
        return -1;
    }
    Motor_SpeedFfFit_Init(&fit);
    (void)Motor_Position_Enable(false);
    (void)Motor_Joint_Enable(false);
    motor.speed_allow_braking = true;
    PRINTF("speed ff detect: REMOVE LOAD, %+.0f RPM, any key aborts\n", max_rpm);
    if (!speed_test_set_rpm(max_rpm / 4.0f) || !speed_ff_set_enable(true)) goto done;
    if (!speed_ff_run_direction(1.0f, max_rpm, &fit)) goto done;
    (void)Motor_RequestStop(&motor);
    osDelay(pdMS_TO_TICKS(500U));
    if (!speed_test_set_rpm(-max_rpm / 4.0f) || !speed_ff_set_enable(true)) goto done;
    if (!speed_ff_run_direction(-1.0f, max_rpm, &fit)) goto done;
    if (!Motor_SpeedFfFit_Solve(&fit, &result) || result.rms_error_v > 0.20f) {
        PRINTF("speed ff detect: fit rejected\n");
        goto done;
    }
    PRINTF("speed ff detect: PASS points=%u\n", result.points);
    PRINTF("  MOTOR_ENCODER_SPEED_FF_V_PER_RPM = %.7f V/RPM\n", result.slope_v_per_rpm);
    PRINTF("  MOTOR_ENCODER_SPEED_FF_BOOST_V   = %.4f V\n", result.boost_v);
    PRINTF("  fit RMS error                    = %.4f V\n", result.rms_error_v);
    PRINTF("Result is not saved; review it before changing the motor profile.\n");
    ok = true;
done:
    (void)Motor_RequestStop(&motor);
    initial_cmd.enable = false;
    (void)Motor_SetCommand(&motor, &initial_cmd);
    motor.speed_allow_braking = initial_braking;
    if (!ok) PRINTF("speed ff detect: FAIL, motor auto-stopped\n");
    return ok ? 0 : -1;
}
static CMD_QUEUE_ITEM s_speed_key;

/**
 * @description: 把已拼好的字节写入串口发送队列。
 * @param {const char *} buf  内容，可为空
 * @param {uint32_t} len  字节数
 * @return {*} 无
 */
static void speed_emit_buf(const char *buf, uint32_t len)
{
    COMMON_ASSERT(buf != NULL);
    if ((len == 0U)) return;
    (void)Bsp_Usart_Write((const UCHAR *)buf, (ULONG)len);
}

/**
 * @description: 发送以 0 结尾的字符串。
 * @param {const char *} text  内容，可为空
 * @return {*} 无
 */
static void speed_emit_str(const char *text)
{
    COMMON_ASSERT(text != NULL);
    speed_emit_buf(text, (uint32_t)strlen(text));
}

/**
 * @description: 测试过程中是否收到任意按键。
 * @detail: 命令任务正占用在转速序列里，主循环不会取队列。读到一包就停止并清掉后续输入。
 * @return {bool} true=要停止
 */
bool Cmd_Speed_StopKey(void)
{
    if (queueCmdHandle == NULL) return false;
    if (osMessageQueueGet(queueCmdHandle, &s_speed_key, NULL, 0) != osOK) return false;
    while (osMessageQueueGet(queueCmdHandle, &s_speed_key, NULL, 0) == osOK) {
    }
    return true;
}

/**
 * @description: 等待转速保持时间，中途有按键则提前返回。
 * @param {uint32_t} period_ms  等待时间，单位 ms
 * @return {bool} true=用户按键
 */
static bool speed_wait_or_key(uint32_t period_ms)
{
    uint32_t waited_ms = 0U;

    while (waited_ms < period_ms) {
        uint32_t step_ms = period_ms - waited_ms;

        if (step_ms > SPEED_TEST_POLL_MS) {
            step_ms = SPEED_TEST_POLL_MS;
        }
        osDelay(pdMS_TO_TICKS(step_ms));
        waited_ms += step_ms;
        if (Cmd_Speed_StopKey()) return true;
    }
    return false;
}
/* end sltest */

/**
 * @description: 按固定两位小数追加转速。
 * @param {textbuf_t *} tb 输出缓冲
 * @param {float} rpm 转速，单位 RPM
 * @return {*} 无
 */
static void speed_append_rpm(textbuf_t *tb, float rpm)
{
    float scaled = rpm * 100.0f;
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
 * @description: 写入一个速度测试目标。
 * @detail: 关掉位置环并打开速度环。运行中禁止正负直接对穿，必须先经过 0。
 * @param {float} rpm  目标转速，单位 RPM
 * @return {bool} true=已写入
 */
static bool speed_test_set_rpm(float rpm)
{
    motor_command_t cmd;
    float ref_rpm;

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return false;
    if (cmd.enable) {
        ref_rpm = (fabsf(motor.ol_current_rpm) > 1.0f) ? motor.ol_current_rpm : cmd.target_rpm;
        if ((rpm != 0.0f) && (ref_rpm != 0.0f) && ((rpm > 0.0f) != (ref_rpm > 0.0f))) {
            speed_emit_str("speed test: pass through 0 before reverse\r\n");
            return false;
        }
    }
    (void)Motor_Position_Enable(false);
    (void)Motor_Joint_Enable(false);
    cmd.mode = MOTOR_OL_MODE_IF;
    cmd.speed_enable = true;
    cmd.target_rpm = rpm;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        speed_emit_str("speed test: invalid speed\r\n");
        return false;
    }
    return true;
}

/**
 * @description: 依次切到预设转速，任意键停止。
 * @detail: 每段之间 osDelay，不关调度器。正转到反转中间有 0 RPM。
 * @param {uint32_t} dwell_ms  每个转速的保持时间，单位 ms
 * @return {*} 无
 */
static void speed_test_run(uint32_t dwell_ms)
{
    char line[96];
    textbuf_t tb;
    unsigned int count = sizeof(s_speed_test_rpm) / sizeof(s_speed_test_rpm[0]);
    unsigned int i;

    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "speed test ");
    (void)textbuf_append_u32(&tb, dwell_ms);
    (void)textbuf_append_str(&tb, " ms, any key stops\r\n");
    speed_emit_buf(line, textbuf_length(&tb));

    for (i = 0U; i < count; i++) {
        float measured_rpm;
        float error_rpm;

        if (!speed_test_set_rpm(s_speed_test_rpm[i])) return;
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_u32(&tb, (uint32_t)(i + 1U));
        (void)textbuf_append_char(&tb, '/');
        (void)textbuf_append_u32(&tb, (uint32_t)count);
        (void)textbuf_append_str(&tb, " target=");
        (void)textbuf_append_i32(&tb, (int32_t)s_speed_test_rpm[i], false);
        (void)textbuf_append_str(&tb, " RPM\r\n");
        speed_emit_buf(line, textbuf_length(&tb));

        if (speed_wait_or_key(dwell_ms)) {
            speed_emit_str("speed test stopped\r\n");
            return;
        }

        measured_rpm = Motor_SpeedLoop_GetMeasuredRpm();
        error_rpm = s_speed_test_rpm[i] - measured_rpm;
        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_str(&tb, "  meas=");
        speed_append_rpm(&tb, measured_rpm);
        (void)textbuf_append_str(&tb, " err=");
        speed_append_rpm(&tb, error_rpm);
        (void)textbuf_append_str(&tb, " RPM\r\n");
        speed_emit_buf(line, textbuf_length(&tb));
    }
    speed_emit_str("speed test done\r\n");
}

/**
 * @description: speed 命令 — 查看速度环状态，或按预设转速切换。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 无参数时打印状态；argv[1] 为 test 时跑转速序列，argv[2] 为保持时间 (ms)
 * @return {int} 0 成功，-1 前置条件不满足
 * @author: lkc
 */
int Cmd_SpeedStatus(int argc, char *argv[])
{
    uint32_t dwell_ms = SPEED_TEST_DWELL_DEFAULT_MS;

    if (argc >= 3 && strcmp(argv[1], "ff") == 0 && strcmp(argv[2], "detect") == 0) {
        float max_rpm = argc >= 4 ? atof(argv[3]) : 1200.0f;
        return speed_ff_detect(max_rpm);
    }
    if ((argc < 2) || (strcmp(argv[1], "test") != 0)) {
        if (argc >= 2) {
            PRINTF("Usage: speed [test [ms] | ff detect [max_rpm]]\n");
            return -1;
        }
        Cmd_Ol_PrintStatus();
        return 0;
    }

    if (argc >= 3) {
        dwell_ms = (uint32_t)atoi(argv[2]);
    }
    if (dwell_ms < SPEED_TEST_DWELL_MIN_MS) {
        dwell_ms = SPEED_TEST_DWELL_MIN_MS;
    }
    if (dwell_ms > SPEED_TEST_DWELL_MAX_MS) {
        dwell_ms = SPEED_TEST_DWELL_MAX_MS;
    }
    if (Motor_Protect_IsFaultLatched()) {
        PRINTF("Clear motor fault before speed test\n");
        return -1;
    }
    if (motor.ol_mode != MOTOR_OL_MODE_IF) {
        PRINTF("Speed test requires I/F mode; run: olmode 1\n");
        return -1;
    }
    if (!motor.sensorless_enable && !Motor_Feedback_IsSensoredSource(Motor_GetFeedbackSource())) {
        PRINTF("Need speed feedback: slenable 1, or olfb encoder|hall\n");
        return -1;
    }
    if (!speed_test_set_rpm(s_speed_test_rpm[0])) return -1;
    if (!motor.ol_enable) {
        PRINTF("Target RPM set to 150 (stopped, use olstart)\n");
        return 0;
    }
    speed_test_run(dwell_ms);
    return 0;
}

/**
 * @description: spenable 命令 — 启停速度 PI 闭环。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 0 或 1
 * @return {int} 0 成功，-1 参数错误或前置条件不满足
 * @author: lkc
 */
int Cmd_SpeedEnable(int argc, char *argv[])
{
    motor_command_t cmd;
    int enable;

    if (argc < 2) {
        PRINTF("Speed loop = %s  (usage: spenable <0|1>)\n", motor.speed_enable ? "ON" : "OFF");
        return 0;
    }

    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }
    if (!enable) {
        (void)Motor_Position_Enable(false);
    }
    if (enable && motor.ol_mode != MOTOR_OL_MODE_IF) {
        PRINTF("Speed loop requires I/F mode; run: olmode 1\n");
        return -1;
    }
    if (enable && Motor_Protect_IsFaultLatched()) {
        PRINTF("Clear motor fault before enabling speed loop\n");
        return -1;
    }
    if (enable && !motor.sensorless_enable && !Motor_Feedback_IsSensoredSource(Motor_GetFeedbackSource())) {
        PRINTF("Need speed feedback: slenable 1, or olfb encoder|hall\n");
        return -1;
    }

    (void)Motor_Joint_Enable(false);
    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    cmd.speed_enable = (enable != 0);
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        PRINTF("Invalid speed-loop command\n");
        return -1;
    }
    PRINTF("Speed loop %s\n", motor.speed_enable ? "enabled" : "disabled");
    if (motor.speed_enable && !Motor_SpeedLoop_IsFeedbackValid()) {
        if (Motor_Feedback_IsSensoredSource(Motor_GetFeedbackSource())) {
            PRINTF("Waiting: encoder Detect / hall table, and speed_valid\n");
        } else {
            PRINTF("I/F startup Iq is used until the sensorless observer locks\n");
        }
    }
    return 0;
}

/**
 * @description: spkp 命令 — 设置速度环比例增益 Kp。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Kp (A/RPM)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SpeedKp(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Speed Kp = %.6f A/RPM  (usage: spkp <value>)\n", motor.speed_kp);
        return 0;
    }
    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_kp = atof(argv[1]);
    if (cfg.speed_kp < 0.0f) {
        cfg.speed_kp = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Kp set to %.6f A/RPM\n", motor.speed_kp);
    return 0;
}

/**
 * @description: spki 命令 — 设置速度环积分增益 Ki。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Ki (A/(RPM·s))
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SpeedKi(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Speed Ki = %.6f A/(RPM*s)  (usage: spki <value>)\n", motor.speed_ki);
        return 0;
    }
    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_ki = atof(argv[1]);
    if (cfg.speed_ki < 0.0f) {
        cfg.speed_ki = 0.0f;
    }
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Ki set to %.6f A/(RPM*s)\n", motor.speed_ki);
    return 0;
}

/**
 * @description: spiq 命令 — 设置速度 PI 可请求的最大 Iq 电流。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 Iq 上限 (A)
 * @return {int} 0 成功
 * @author: lkc
 */
int Cmd_SpeedIqLimit(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;

    if (argc < 2) {
        PRINTF("Speed Iq limit = %.3f A  (usage: spiq <amps>)\n", motor.speed_iq_limit);
        return 0;
    }
    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_iq_limit = fabsf(atof(argv[1]));
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed Iq limit set to %.3f A\n", motor.speed_iq_limit);
    return 0;
}

/**
 * @description: spbrake 命令 — 允许或禁止速度 PI 输出反向制动电流。
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为 0 或 1
 * @return {int} 0 成功，-1 参数无效
 * @author: lkc
 */
int Cmd_SpeedBrake(int argc, char *argv[])
{
    motor_config_t cfg;
    motor_result_t result;
    int enable;

    if (argc < 2) {
        PRINTF("Speed braking = %s  (usage: spbrake <0|1>)\n", motor.speed_allow_braking ? "ON" : "OFF");
        return 0;
    }
    enable = atoi(argv[1]);
    if (enable != 0 && enable != 1) {
        PRINTF("Invalid value, use 0 or 1\n");
        return -1;
    }
    if (!Cmd_Ol_CaptureConfig(&cfg)) {
        PRINTF("Invalid config\n");
        return -1;
    }
    cfg.speed_allow_braking = (enable != 0);
    result = Motor_SetConfig(&motor, &cfg);
    if (result != MOTOR_OK) {
        Cmd_Ol_PrintConfigResult(result);
        return -1;
    }
    Motor_SpeedLoop_Reset();
    PRINTF("Speed braking %s\n", motor.speed_allow_braking ? "enabled" : "disabled");
    return 0;
}
#ifdef __cplusplus
}
#endif
