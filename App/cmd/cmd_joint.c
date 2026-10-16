/*
 * @Description: 关节阻抗 Shell 命令，与 pos / olspeed 分开
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "common_assert.h"
/* Private define ------------------------------------------------------------*/
#define JOINT_TEST_DWELL_DEFAULT_MS 2000U
#define JOINT_TEST_DWELL_MIN_MS     500U
#define JOINT_TEST_DWELL_MAX_MS     10000U
#define JOINT_TEST_POLL_MS          50U
#define JOINT_SLOW_SPEED_DEFAULT_DEG_S (10.0f)
#define JOINT_SLOW_SPEED_MIN_DEG_S     (0.5f)
#define JOINT_SLOW_SPEED_MAX_DEG_S     (90.0f)
#define JOINT_SLOW_UPDATE_MS            20U
#define JOINT_SLOW_SETTLE_MS            500U
#define JOINT_IQ_TEST_DEFAULT_MS       1000U
#define JOINT_IQ_TEST_MIN_MS            100U
#define JOINT_IQ_TEST_MAX_MS           5000U
#define JOINT_IQ_TEST_REFRESH_MS         20U
#define JOINT_IQ_TEST_MAX_A            (0.30f) /* 覆盖 0.25 A 混合后端进入阈值；仅限受限测试命令 */
#define JOINT_HYBRID_TEST_IQ_LIMIT_A     (0.25f)
#define JOINT_HYBRID_TEST_DWELL_MS       1000U
#define JOINT_HYBRID_TEST_TOL_DEG        (0.75f)
#define JOINT_HYBRID_POLL_SETTLED        0
#define JOINT_HYBRID_POLL_STOPPED        1
#define JOINT_HYBRID_POLL_TIMEOUT        2
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const float s_joint_test_deg[] = {0.0f, 30.0f, 90.0f, 180.0f, 90.0f, 0.0f, -90.0f, 0.0f};
/* 相对起点。+9/+30 是小角，+90/+180/+360 是大角。 */
static const float s_joint_hybrid_delta_deg[] = {9.0f, 30.0f, 90.0f, 180.0f, 360.0f, 0.0f, -180.0f, 0.0f};
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
static void joint_slow_run(float speed_deg_s);
static void joint_iq_test_run(float current_a, uint32_t duration_ms);
static void joint_hybrid_note_switch(const motor_joint_state_t *state, uint32_t enter0, uint32_t exit0, bool *saw_iq,
                                     bool *saw_vq);
static int joint_hybrid_poll(float goal_deg, bool require_switch, uint32_t timeout_ms, uint32_t enter0,
                             uint32_t exit0, bool *saw_iq, bool *saw_vq);
static void joint_hybrid_hold_here(void);
static void joint_hybrid_emit_step(unsigned int index, unsigned int count, float target_deg, int rc, bool switch_ok);
static void joint_hybrid_test_run(float rpm);
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
    COMMON_ASSERT(buf != NULL);
    if ((len == 0U)) return;
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
    COMMON_ASSERT(text != NULL);
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
    if (queueCmdHandle == NULL) return false;
    if (osMessageQueueGet(queueCmdHandle, &s_joint_key, NULL, 0) != osOK) return false;
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
        if (joint_stop_key()) return true;
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
        (void)textbuf_append_str(&tb, " iq=");
        joint_append_fixed(&tb, state.iq_command_a);
        (void)textbuf_append_str(&tb, " bias=");
        joint_append_fixed(&tb, state.bias_v);
        (void)textbuf_append_str(&tb, " A\r\n");
        joint_emit_buf(line, textbuf_length(&tb));
    }
    joint_emit_str("joint test done\r\n");
}

/**
 * @description: 按恒定目标速度逐步推进预设位置，形成慢速斜坡测试。
 * @detail: 每20 ms更新一次位置目标，同时给出同方向速度前馈；到达每个目标后以零目标速度保持500 ms。
 *          任意键停止时把当前位置设为新的保持目标，避免继续追赶尚未到达的轨迹点。
 * @param {float} speed_deg_s  轨迹速度绝对值 (deg/s)
 * @return {void} 无
 */
static void joint_slow_run(float speed_deg_s)
{
    char line[112];
    textbuf_t tb;
    motor_joint_state_t state;
    float command_deg;
    float step_deg = speed_deg_s * (float)JOINT_SLOW_UPDATE_MS / 1000.0f;
    unsigned int count = sizeof(s_joint_test_deg) / sizeof(s_joint_test_deg[0]);
    unsigned int i;

    Motor_Joint_GetState(&state);
    command_deg = state.position_deg;
    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "joint slow ");
    joint_append_fixed(&tb, speed_deg_s);
    (void)textbuf_append_str(&tb, " deg/s, any key stops\r\n");
    joint_emit_buf(line, textbuf_length(&tb));

    for (i = 0U; i < count; i++) {
        const float destination_deg = s_joint_test_deg[i];

        textbuf_init(&tb, line, sizeof(line));
        (void)textbuf_append_u32(&tb, (uint32_t)(i + 1U));
        (void)textbuf_append_char(&tb, '/');
        (void)textbuf_append_u32(&tb, (uint32_t)count);
        (void)textbuf_append_str(&tb, " ramp target=");
        joint_append_fixed(&tb, destination_deg);
        (void)textbuf_append_str(&tb, " deg\r\n");
        joint_emit_buf(line, textbuf_length(&tb));

        while (fabsf(destination_deg - command_deg) > step_deg) {
            float direction = (destination_deg > command_deg) ? 1.0f : -1.0f;
            float target_rpm = direction * speed_deg_s / 6.0f;

            command_deg += direction * step_deg;
            if (!Motor_Joint_SetTarget(command_deg, target_rpm, 0.0f)) {
                joint_emit_str("joint slow: bad target\r\n");
                return;
            }
            if (joint_wait_or_key(JOINT_SLOW_UPDATE_MS)) {
                Motor_Joint_GetState(&state);
                (void)Motor_Joint_SetTarget(state.position_deg, 0.0f, 0.0f);
                joint_emit_str("joint slow stopped, holding current position\r\n");
                return;
            }
        }
        command_deg = destination_deg;
        if (!Motor_Joint_SetTarget(command_deg, 0.0f, 0.0f)) {
            joint_emit_str("joint slow: bad target\r\n");
            return;
        }
        if (joint_wait_or_key(JOINT_SLOW_SETTLE_MS)) {
            Motor_Joint_GetState(&state);
            (void)Motor_Joint_SetTarget(state.position_deg, 0.0f, 0.0f);
            joint_emit_str("joint slow stopped, holding current position\r\n");
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
        (void)textbuf_append_str(&tb, " iq=");
        joint_append_fixed(&tb, state.iq_command_a);
        (void)textbuf_append_str(&tb, " A\r\n");
        joint_emit_buf(line, textbuf_length(&tb));
    }
    joint_emit_str("joint slow done\r\n");
}

/**
 * @description: 短时刷新直接Iq命令，结束后立即关闭关节并停机。
 * @detail: 每20 ms刷新100 ms超时保护；按键、提交失败或到时都会退出直接电流模式。
 *          空载转子可能已被加速，不能在结束瞬间切到位置保持，否则会追赶高速转子并反向回摆。
 * @param {float} current_a  目标 Iq (A)
 * @param {uint32_t} duration_ms  持续时间 (ms)
 * @return {void} 无
 * @author: lkc
 */
static void joint_iq_test_run(float current_a, uint32_t duration_ms)
{
    motor_joint_current_command_t command;
    motor_current_pi_diag_t pi_diag;
    motor_joint_state_t state;
    uint32_t elapsed_ms = 0U;
    bool stopped = false;

    memset(&command, 0, sizeof(command));
    command.enable = true;
    command.timeout_enable = true;
    command.timeout_us = 100000U;
    command.current_a = current_a;
    command.current_limit_a = JOINT_IQ_TEST_MAX_A;
    /* 丢弃测试前的PI统计，保证结果只覆盖本次固定Iq窗口。 */
    Motor_Current_TakePiDiag(&pi_diag);
    PRINTF("joint iqtest %.3fA %lums, any key stops\r\n", current_a, (unsigned long)duration_ms);

    while (elapsed_ms < duration_ms) {
        uint32_t step_ms = duration_ms - elapsed_ms;

        if (step_ms > JOINT_IQ_TEST_REFRESH_MS) {
            step_ms = JOINT_IQ_TEST_REFRESH_MS;
        }
        if (!Motor_Joint_SubmitCurrentCommand(&command)) {
            PRINTF("joint iqtest: command rejected\r\n");
            stopped = true;
            break;
        }
        if (joint_wait_or_key(step_ms)) {
            stopped = true;
            break;
        }
        elapsed_ms += step_ms;
    }

    Motor_Current_TakePiDiag(&pi_diag);
    Motor_Joint_GetState(&state);
    PRINTF("joint iqtest %s: pos=%.2fdeg vel=%.2fRPM iq_cmd=%.3fA iq_avg=%.3fA "
           "err_avg=%.3fA err_abs=%.3fA vq_int=%.3fV samples=%lu; auto-stopped\r\n",
           stopped ? "stopped" : "done", state.position_deg, state.measured_rpm, current_a,
           current_a - pi_diag.iq_err_avg, pi_diag.iq_err_avg, pi_diag.iq_err_abs_avg,
           motor.ol_vq_int, (unsigned long)pi_diag.samples);
    (void)Motor_Joint_Enable(false);
    (void)Motor_RequestStop(&motor);
}

/**
 * @description: 单次提交直接 Iq 命令，等待看门狗超时并打印超时后的安全输出。
 * @detail: 用于上板验证通信停止刷新后的 100 ms 保护；测试结束关闭关节并停机。
 * @param {float} current_a  超时前的目标 Iq (A)
 * @param {uint32_t} wait_ms 等待时间 (ms)，必须大于看门狗时间
 * @return {void} 无
 */
static void joint_timeout_test_run(float current_a, uint32_t wait_ms)
{
    motor_joint_current_command_t command;
    motor_joint_state_t state;

    memset(&command, 0, sizeof(command));
    command.enable = true;
    command.timeout_enable = true;
    command.timeout_us = 100000U;
    command.current_a = current_a;
    command.current_limit_a = JOINT_IQ_TEST_MAX_A;

    PRINTF("joint timeouttest submit %.3fA once, wait %lums\r\n", current_a, (unsigned long)wait_ms);
    if (!Motor_Joint_SubmitCurrentCommand(&command)) {
        PRINTF("joint timeouttest: command rejected\r\n");
        return;
    }
    if (joint_wait_or_key(wait_ms)) {
        PRINTF("joint timeouttest stopped by key\r\n");
    }
    Motor_Joint_GetState(&state);
    PRINTF("joint timeouttest result: timeout=%d age=%luus backend=%d iq_cmd=%.3fA vel=%.2fRPM\r\n",
           state.command_timeout ? 1 : 0, (unsigned long)state.command_age_us, (int)state.backend,
           state.iq_command_a, state.measured_rpm);
    (void)Motor_Joint_Enable(false);
    (void)Motor_RequestStop(&motor);
    PRINTF("joint timeouttest cleanup: motor auto-stopped\r\n");
}

/**
 * @description: 记录本段运动里是否出现过 IQ，以及随后回到 VQ。
 * @param {const motor_joint_state_t *} state  当前关节状态
 * @param {uint32_t} enter0  本段开始时的进入次数
 * @param {uint32_t} exit0  本段开始时的退出次数
 * @param {bool *} saw_iq  见过 IQ 区时置 true
 * @param {bool *} saw_vq  IQ 之后回到 VQ 时置 true
 * @return {void} 无
 * @author: lkc
 */
static void joint_hybrid_note_switch(const motor_joint_state_t *state, uint32_t enter0, uint32_t exit0, bool *saw_iq,
                                     bool *saw_vq)
{
    if (state->hybrid_current_active || state->hybrid_enter_count > enter0) {
        *saw_iq = true;
    }
    if (*saw_iq && state->hybrid_exit_count > exit0 && !state->hybrid_current_active) {
        *saw_vq = true;
    }
}

/**
 * @description: 按这段路程和转速估算等待上限。
 * @detail: 时间 = 路程 / 转速 + 0.5 s 加速 + 到位停留 + 2 s 余量。单段最长 1 小时，避免极低转速占住串口。
 * @param {float} distance_deg  本段角度 (deg)
 * @param {float} rpm  轨迹最高转速 (RPM)
 * @return {uint32_t} 等待上限 (ms)
 * @author: lkc
 */
static uint32_t joint_hybrid_move_timeout_ms(float distance_deg, float rpm)
{
    float vel_deg_s = rpm * 6.0f;
    float seconds;

    if (!(vel_deg_s > 0.0f) || !isfinite(vel_deg_s) || !isfinite(distance_deg)) return JOINT_HYBRID_TEST_DWELL_MS + 2000U;
    seconds = fabsf(distance_deg) / vel_deg_s + 0.5f + (JOINT_HYBRID_TEST_DWELL_MS / 1000.0f) + 2.0f;
    if (!isfinite(seconds) || seconds < 1.0f) {
        seconds = 1.0f;
    }
    if (seconds > 3600.0f) {
        seconds = 3600.0f;
    }
    return (uint32_t)(seconds * 1000.0f);
}

/**
 * @description: 等待到达目标，去程同时等待 IQ 回到 VQ。
 * @param {float} goal_deg  目标角度 (deg)
 * @param {bool} require_switch  true=必须先看到 IQ→VQ 才算到位
 * @param {uint32_t} timeout_ms  本段等待上限 (ms)
 * @param {uint32_t} enter0  本段开始时的进入次数
 * @param {uint32_t} exit0  本段开始时的退出次数
 * @param {bool *} saw_iq  切换观察，可累计
 * @param {bool *} saw_vq  切换观察，可累计
 * @return {int} 0=到位，1=按键停止，2=超时
 * @author: lkc
 */
static int joint_hybrid_poll(float goal_deg, bool require_switch, uint32_t timeout_ms, uint32_t enter0, uint32_t exit0,
                             bool *saw_iq, bool *saw_vq)
{
    uint32_t waited_ms = 0U;
    uint32_t settled_ms = 0U;

    while (waited_ms < timeout_ms) {
        motor_joint_state_t state;
        bool switch_ready;

        if (joint_wait_or_key(JOINT_TEST_POLL_MS)) return JOINT_HYBRID_POLL_STOPPED;
        waited_ms += JOINT_TEST_POLL_MS;
        Motor_Joint_GetState(&state);
        joint_hybrid_note_switch(&state, enter0, exit0, saw_iq, saw_vq);
        switch_ready = !require_switch || !*saw_iq || *saw_vq;

        if (fabsf(state.position_deg - goal_deg) <= JOINT_HYBRID_TEST_TOL_DEG && switch_ready) {
            settled_ms += JOINT_TEST_POLL_MS;
            if (settled_ms >= JOINT_HYBRID_TEST_DWELL_MS) return JOINT_HYBRID_POLL_SETTLED;
        } else {
            settled_ms = 0U;
        }
    }
    return JOINT_HYBRID_POLL_TIMEOUT;
}

/**
 * @description: 按键停止后把目标改成当前角度，关节停在原地。
 * @return {void} 无
 * @author: lkc
 */
static void joint_hybrid_hold_here(void)
{
    motor_joint_state_t state;

    Motor_Joint_GetState(&state);
    (void)Motor_Joint_SetTarget(state.position_deg, 0.0f, 0.0f);
}

/**
 * @description: 打印一个混合测试位置的结果。
 * @param {unsigned int} index  从 0 开始的位置序号
 * @param {unsigned int} count  位置总数
 * @param {float} target_deg  该点目标角度 (deg)
 * @param {int} rc  等待结果，0=到位，1=按键，2=超时
 * @param {bool} switch_ok  true=本段看到 IQ 回到 VQ
 * @return {void} 无
 * @author: lkc
 */
static void joint_hybrid_emit_step(unsigned int index, unsigned int count, float target_deg, int rc, bool switch_ok)
{
    motor_joint_state_t state;
    char line[128];
    textbuf_t tb;
    const char *status = "timeout";

    if (rc == JOINT_HYBRID_POLL_SETTLED) {
        status = "ok";
    } else if (rc == JOINT_HYBRID_POLL_STOPPED) {
        status = "stopped";
    }
    Motor_Joint_GetState(&state);
    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_u32(&tb, (uint32_t)(index + 1U));
    (void)textbuf_append_char(&tb, '/');
    (void)textbuf_append_u32(&tb, (uint32_t)count);
    (void)textbuf_append_str(&tb, " target=");
    joint_append_fixed(&tb, target_deg);
    (void)textbuf_append_char(&tb, ' ');
    (void)textbuf_append_str(&tb, status);
    (void)textbuf_append_str(&tb, " iq_to_vq=");
    (void)textbuf_append_str(&tb, switch_ok ? "yes" : "no");
    (void)textbuf_append_str(&tb, " pos=");
    joint_append_fixed(&tb, state.position_deg);
    (void)textbuf_append_str(&tb, " err=");
    joint_append_fixed(&tb, state.error_deg);
    (void)textbuf_append_str(&tb, "\r\n");
    joint_emit_buf(line, textbuf_length(&tb));
}

/**
 * @description: 从当前位置走小角和大角，可临时改轨迹最高转速。
 * @detail: 每段必须在误差窗内连续稳定；若运动确实进入IQ区，则必须在到位前回到VQ。
 *          轻载轨迹允许全程VQ，不能为覆盖测试而强迫BM75在低计数区使用电流环。
 *          rpm 大于 0 时只在本次测试内改轨迹转速，结束恢复。
 *          临时把speed_iq_limit提高到0.25A；结束、超时或按键后都恢复并自动停机。
 * @param {float} rpm  轨迹最高转速 (RPM)。小于等于 0 用头文件默认
 * @return {void} 无
 * @author: lkc
 */
static void joint_hybrid_test_run(float rpm)
{
    motor_settings_t settings;
    motor_config_t test_config;
    motor_joint_state_t before;
    char line[192];
    textbuf_t tb;
    float origin_deg;
    unsigned int count = sizeof(s_joint_hybrid_delta_deg) / sizeof(s_joint_hybrid_delta_deg[0]);
    unsigned int index;
    unsigned int passed = 0U;
    bool limit_changed = false;
    bool limit_restored = true;
    bool rpm_changed = false;
    bool stopped = false;
    bool failed = false;
    const char *result = "pass";

    if (!Motor_Settings_Capture(&motor, &settings)) {
        joint_emit_str("joint hybridtest: cannot capture motor config\r\n");
        return;
    }
    if (rpm > 0.0f) {
        Motor_Joint_SetTrajectoryRpm(rpm);
        rpm_changed = true;
    }
    test_config = settings.config;
    Motor_Joint_GetState(&before);
    origin_deg = before.position_deg;
    if (test_config.speed_iq_limit < JOINT_HYBRID_TEST_IQ_LIMIT_A) {
        test_config.speed_iq_limit = JOINT_HYBRID_TEST_IQ_LIMIT_A;
        if (Motor_SetConfig(&motor, &test_config) != MOTOR_OK) {
            if (rpm_changed) {
                Motor_Joint_SetTrajectoryRpm(0.0f);
            }
            joint_emit_str("joint hybridtest: cannot set temporary Iq limit\r\n");
            return;
        }
        limit_changed = true;
    }

    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "joint hybridtest ");
    joint_append_fixed(&tb, Motor_Joint_GetTrajectoryRpm());
    (void)textbuf_append_str(&tb, " RPM: +9/+30/+90/+180/+360/0/-180/0 from ");
    joint_append_fixed(&tb, origin_deg);
    (void)textbuf_append_str(&tb, " deg, Iq limit ");
    joint_append_fixed(&tb, motor.speed_iq_limit);
    (void)textbuf_append_str(&tb, "A, settle 1000 ms within 0.75 deg, any key stops\r\n");
    joint_emit_buf(line, textbuf_length(&tb));

    for (index = 0U; index < count; index++) {
        float goal_deg = origin_deg + s_joint_hybrid_delta_deg[index];
        uint32_t enter0;
        uint32_t exit0;
        bool saw_iq;
        bool saw_vq = false;
        int rc;

        Motor_Joint_GetState(&before);
        enter0 = before.hybrid_enter_count;
        exit0 = before.hybrid_exit_count;
        saw_iq = before.hybrid_current_active;
        if (!Motor_Joint_SetTarget(goal_deg, 0.0f, 0.0f)) {
            joint_hybrid_emit_step(index, count, goal_deg, JOINT_HYBRID_POLL_TIMEOUT, false);
            failed = true;
            break;
        }
        rc = joint_hybrid_poll(goal_deg, true,
                               joint_hybrid_move_timeout_ms(goal_deg - before.position_deg,
                                                            Motor_Joint_GetTrajectoryRpm()),
                               enter0, exit0, &saw_iq, &saw_vq);
        joint_hybrid_emit_step(index, count, goal_deg, rc, saw_vq);
        if (rc == JOINT_HYBRID_POLL_STOPPED) {
            stopped = true;
            break;
        }
        if (rc != JOINT_HYBRID_POLL_SETTLED) {
            failed = true;
            break;
        }
        passed++;
    }

    if (stopped) {
        joint_hybrid_hold_here();
        result = "stopped";
    } else if (failed) {
        Motor_Joint_GetState(&before);
        if (fabsf(before.target_deg - origin_deg) > 0.5f) {
            uint32_t enter0 = before.hybrid_enter_count;
            uint32_t exit0 = before.hybrid_exit_count;
            bool saw_iq = before.hybrid_current_active;
            bool saw_vq = false;
            int rc;

            if (Motor_Joint_SetTarget(origin_deg, 0.0f, 0.0f)) {
                rc = joint_hybrid_poll(origin_deg, false,
                                       joint_hybrid_move_timeout_ms(origin_deg - before.position_deg,
                                                                    Motor_Joint_GetTrajectoryRpm()),
                                       enter0, exit0, &saw_iq, &saw_vq);
                if (rc == JOINT_HYBRID_POLL_STOPPED) {
                    joint_hybrid_hold_here();
                    stopped = true;
                    result = "stopped";
                }
            }
        }
        if (!stopped) {
            result = "fail";
        }
    }

    if (limit_changed && Motor_SetConfig(&motor, &settings.config) != MOTOR_OK) {
        limit_restored = false;
    }
    if (rpm_changed) {
        Motor_Joint_SetTrajectoryRpm(0.0f);
    }
    Motor_Joint_GetState(&before);
    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "joint hybridtest ");
    (void)textbuf_append_str(&tb, result);
    (void)textbuf_append_str(&tb, ": ");
    (void)textbuf_append_u32(&tb, (uint32_t)passed);
    (void)textbuf_append_char(&tb, '/');
    (void)textbuf_append_u32(&tb, (uint32_t)count);
    (void)textbuf_append_str(&tb, " pos=");
    joint_append_fixed(&tb, before.position_deg);
    (void)textbuf_append_str(&tb, " err=");
    joint_append_fixed(&tb, before.error_deg);
    (void)textbuf_append_str(&tb, " mode=");
    (void)textbuf_append_str(&tb, before.hybrid_current_active ? "IQ" : "VQ");
    (void)textbuf_append_str(&tb, " limit=");
    joint_append_fixed(&tb, motor.speed_iq_limit);
    (void)textbuf_append_str(&tb, limit_restored ? "A restored\r\n" : "A restore failed\r\n");
    joint_emit_buf(line, textbuf_length(&tb));
    (void)Motor_Joint_Enable(false);
    (void)Motor_RequestStop(&motor);
    joint_emit_str("joint hybridtest motor auto-stopped\r\n");
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
    PRINTF("Joint: en=%d pos=%.2f target=%.2f err=%.2f plan=%.2f/%.1fRPM traj=%d vel=%.1f vdes=%.1f "
           "tff=%.3fA iq_cmd=%.3fA out=%.3fV mode=%s sw=%lu/%lu peak=%.3fA kp=%.3f kd=%.3f ki=%.3f bias=%.3fA lim=%.2fV backend=%d age=%luus watchdog=%d timeout=%d\r\n",
           state.enabled ? 1 : 0, state.position_deg, state.target_deg, state.error_deg,
           state.planned_position_deg, state.planned_rpm, state.trajectory_active ? 1 : 0, state.measured_rpm,
           state.target_rpm, state.torque_ff_a, state.iq_command_a, state.output_v,
           state.hybrid_current_active ? "IQ" : "VQ", (unsigned long)state.hybrid_enter_count,
           (unsigned long)state.hybrid_exit_count, state.hybrid_peak_iq_a,
           state.kp_v_per_deg, state.kd_v_per_rpm,
           state.ki_v_per_deg_s, state.bias_v, state.v_limit_v, (int)state.backend,
           (unsigned long)state.command_age_us,
           state.timeout_enabled ? 1 : 0, state.command_timeout ? 1 : 0);
}

/**
 * @description: 切到 I/F 并打开速度环，转速命令保持 0，避免和阻抗电压叠加。
 * @return {int} 0=已写入，-1=命令被拒绝
 * @author: lkc
 */
static int Cmd_JointArm(void)
{
    motor_command_t cmd;

    if (Motor_GetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    cmd.mode = MOTOR_OL_MODE_IF;
    cmd.speed_enable = true;
    cmd.target_rpm = 0.0f;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) return -1;
    return 0;
}

/**
 * @description: 关节阻抗命令 joint。
 * @detail: 与 pos、olspeed 互斥。set 的三个数是目标角度 (deg)、目标转速 (RPM)、力矩前馈 (A)。
 *          首次使能前电机须停转。尚未上电时提示 olstart。
 *          test 在已上电后按预设角度切换；slow 按指定deg/s生成慢速位置斜坡；
 *          hybridtest [rpm] 从当前位置走 +9、+30、+90、+180、+360、0、-180 deg 再回起点。
 *          不写 rpm 用头文件默认；写入正数只改本次轨迹最高转速。每段打印误差。
 *          iqtest 仅用于短时验证Iq电流路径，结束后自动停机。
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
    bool run_slow = false;
    bool run_iq_test = false;
    bool run_timeout_test = false;
    bool run_hybrid_test = false;
    float slow_speed_deg_s = JOINT_SLOW_SPEED_DEFAULT_DEG_S;
    float iq_test_a = 0.0f;
    float hybrid_rpm = 0.0f;
    uint32_t iq_test_ms = JOINT_IQ_TEST_DEFAULT_MS;
    uint32_t timeout_test_ms = 180U;
    uint32_t dwell_ms = JOINT_TEST_DWELL_DEFAULT_MS;

    if (argc < 2 || strcmp(argv[1], "status") == 0) {
        Cmd_JointPrint();
        PRINTF("usage: joint zero|on|off|set <deg> [rpm] [A]|test [ms]|slow [deg/s]|hybridtest [rpm]|iqtest <A> [ms]|timeouttest <A> [ms]|kp <V/deg>|kd <V/RPM>|ki <V/deg/s>|lim <V>\r\n");
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
    } else if (strcmp(argv[1], "slow") == 0) {
        if (argc >= 3) {
            slow_speed_deg_s = (float)atof(argv[2]);
        }
        if (!isfinite(slow_speed_deg_s) || slow_speed_deg_s < JOINT_SLOW_SPEED_MIN_DEG_S ||
            slow_speed_deg_s > JOINT_SLOW_SPEED_MAX_DEG_S) {
            PRINTF("joint slow speed must be %.1f..%.1f deg/s\r\n", JOINT_SLOW_SPEED_MIN_DEG_S,
                   JOINT_SLOW_SPEED_MAX_DEG_S);
            return -1;
        }
        (void)Motor_Position_Enable(false);
        if (!Motor_Joint_SetTarget(state.position_deg, 0.0f, 0.0f) || !Motor_Joint_Enable(true)) {
            PRINTF("Joint impedance needs valid calibrated encoder\r\n");
            return -1;
        }
        run_slow = true;
    } else if (strcmp(argv[1], "iqtest") == 0 && argc >= 3) {
        iq_test_a = (float)atof(argv[2]);
        if (argc >= 4) {
            iq_test_ms = (uint32_t)atoi(argv[3]);
        }
        if (!isfinite(iq_test_a) || fabsf(iq_test_a) > JOINT_IQ_TEST_MAX_A ||
            fabsf(iq_test_a) < 0.001f) {
            PRINTF("joint iqtest current must be -%.2f..-0.001 or 0.001..%.2f A\r\n",
                   JOINT_IQ_TEST_MAX_A, JOINT_IQ_TEST_MAX_A);
            return -1;
        }
        if (iq_test_ms < JOINT_IQ_TEST_MIN_MS || iq_test_ms > JOINT_IQ_TEST_MAX_MS) {
            PRINTF("joint iqtest duration must be %lu..%lu ms\r\n", (unsigned long)JOINT_IQ_TEST_MIN_MS,
                   (unsigned long)JOINT_IQ_TEST_MAX_MS);
            return -1;
        }
        if (!motor.ol_enable || !Motor_Joint_IsEnabled()) {
            PRINTF("Start joint first: joint on, then olstart\r\n");
            return -1;
        }
        run_iq_test = true;
    } else if (strcmp(argv[1], "timeouttest") == 0 && argc >= 3) {
        iq_test_a = (float)atof(argv[2]);
        if (argc >= 4) {
            timeout_test_ms = (uint32_t)atoi(argv[3]);
        }
        if (!isfinite(iq_test_a) || fabsf(iq_test_a) > JOINT_IQ_TEST_MAX_A ||
            fabsf(iq_test_a) < 0.001f) {
            PRINTF("joint timeouttest current must be -%.2f..-0.001 or 0.001..%.2f A\r\n",
                   JOINT_IQ_TEST_MAX_A, JOINT_IQ_TEST_MAX_A);
            return -1;
        }
        if (timeout_test_ms < 120U || timeout_test_ms > 1000U) {
            PRINTF("joint timeouttest wait must be 120..1000 ms\r\n");
            return -1;
        }
        if (!motor.ol_enable || !Motor_Joint_IsEnabled()) {
            PRINTF("Start joint first: joint on, then olstart\r\n");
            return -1;
        }
        run_timeout_test = true;
    } else if (strcmp(argv[1], "hybridtest") == 0) {
        if (argc >= 3) {
            hybrid_rpm = (float)atof(argv[2]);
            if (!isfinite(hybrid_rpm) || hybrid_rpm <= 0.0f) {
                PRINTF("joint hybridtest rpm must be > 0\r\n");
                return -1;
            }
        }
        if (!motor.ol_enable || !Motor_Joint_IsEnabled()) {
            PRINTF("Start joint first: joint on, then olstart\r\n");
            return -1;
        }
        run_hybrid_test = true;
    } else if (strcmp(argv[1], "kp") == 0 && argc >= 3) {
        if (!Motor_Joint_SetKp((float)atof(argv[2]))) return -1;
        Cmd_JointPrint();
        return 0;
    } else if (strcmp(argv[1], "kd") == 0 && argc >= 3) {
        if (!Motor_Joint_SetKd((float)atof(argv[2]))) return -1;
        Cmd_JointPrint();
        return 0;
    } else if (strcmp(argv[1], "ki") == 0 && argc >= 3) {
        if (!Motor_Joint_SetKi((float)atof(argv[2]))) return -1;
        Cmd_JointPrint();
        return 0;
    } else if (strcmp(argv[1], "lim") == 0 && argc >= 3) {
        if (!Motor_Joint_SetVoltageLimit((float)atof(argv[2]))) return -1;
        Cmd_JointPrint();
        return 0;
    } else {
        PRINTF("usage: joint zero|on|off|set <deg> [rpm] [A]|test [ms]|slow [deg/s]|hybridtest [rpm]|iqtest <A> [ms]|timeouttest <A> [ms]|kp <V/deg>|kd <V/RPM>|ki <V/deg/s>|lim <V>\r\n");
        return -1;
    }
    if (Cmd_JointArm() != 0) return -1;
    Cmd_JointPrint();
    if (!motor.ol_enable) {
        PRINTF("Use olstart to energize joint impedance\r\n");
        return 0;
    }
    if (run_test) {
        joint_test_run(dwell_ms);
    } else if (run_slow) {
        joint_slow_run(slow_speed_deg_s);
    } else if (run_iq_test) {
        joint_iq_test_run(iq_test_a, iq_test_ms);
    } else if (run_timeout_test) {
        joint_timeout_test_run(iq_test_a, timeout_test_ms);
    } else if (run_hybrid_test) {
        joint_hybrid_test_run(hybrid_rpm);
    }
    return 0;
}

#ifdef __cplusplus
}
#endif
