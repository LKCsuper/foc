/*
 * @Description: FreeRTOS 任务状态查看命令
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2026-10-01
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "common_assert.h"
/* Private define ------------------------------------------------------------*/
#define TASK_STAT_SLOTS     20U
#define TASK_WATCH_MIN_MS   200U
#define TASK_WATCH_MAX_MS   5000U
#define TASK_WATCH_STEP_MS  50U
#define TASK_NAME_WIDTH     16U
#define TASK_COL_STATE      2U
#define TASK_COL_PRI        3U
#define TASK_COL_STACK      6U
#define TASK_COL_CPU        6U
#define TASK_TABLE_WIDTH    (TASK_NAME_WIDTH + 1U + TASK_COL_STATE + 1U + TASK_COL_PRI + 1U + TASK_COL_STACK + 1U + TASK_COL_CPU)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TaskStatus_t s_task_now[TASK_STAT_SLOTS];
static TaskStatus_t s_task_prev[TASK_STAT_SLOTS];
static CMD_QUEUE_ITEM s_task_key;
/* Private function prototypes -----------------------------------------------*/
static void task_emit(const char *buf, uint32_t len);
static void task_emit_str(const char *text);
static bool task_stop_key(void);
static bool task_wait_or_key(uint32_t period_ms);
static char task_state_char(eTaskState state);
static uint32_t task_run_delta(const TaskStatus_t *prev, UBaseType_t nprev, UBaseType_t number, uint32_t now);
static uint32_t task_percent_tenths(uint32_t part, uint32_t total);
static void task_append_spaces(textbuf_t *tb, uint32_t count);
static void task_append_left(textbuf_t *tb, const char *text, uint32_t width);
static void task_append_u32_right(textbuf_t *tb, uint32_t value, uint32_t width);
static void task_append_u32_suffix(textbuf_t *tb, uint32_t value, char suffix, uint32_t width);
static void task_append_cpu(textbuf_t *tb, bool valid, uint32_t tenths);
static void task_fill_stack(TaskStatus_t *tasks, UBaseType_t count);
static void task_print_frame(const TaskStatus_t *now, UBaseType_t nnow, const TaskStatus_t *prev, UBaseType_t nprev,
                             uint32_t total_now, uint32_t total_prev, bool use_delta);
static void task_watch(uint32_t period_ms);
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 把一行写进串口发送队列。
 * @param {const char *} buf  内容，可为空
 * @param {uint32_t} len  字节数
 * @return {*} 无
 */
static void task_emit(const char *buf, uint32_t len)
{
    COMMON_ASSERT(buf != NULL);
    if ((len == 0U)) return;
    (void)Bsp_Usart_Write((const UCHAR *)buf, len);
}

/**
 * @description: 发送以 0 结尾的字符串。
 * @param {const char *} text  内容，可为空
 * @return {*} 无
 */
static void task_emit_str(const char *text)
{
    COMMON_ASSERT(text != NULL);
    task_emit(text, (uint32_t)strlen(text));
}

/**
 * @description: 命令队列里是否已经有按键。
 * @detail: 命令任务自己在看统计时，主循环不会取队列。读到任意一包就停止，并清掉后续输入。
 * @return {bool} true=要停止
 */
static bool task_stop_key(void)
{
    if (queueCmdHandle == NULL) return false;
    if (osMessageQueueGet(queueCmdHandle, &s_task_key, NULL, 0) != osOK) return false;
    while (osMessageQueueGet(queueCmdHandle, &s_task_key, NULL, 0) == osOK) {
    }
    return true;
}

/**
 * @description: 等待一个刷新周期，中途有按键则提前返回。
 * @param {uint32_t} period_ms  周期，单位 ms
 * @return {bool} true=用户按键
 */
static bool task_wait_or_key(uint32_t period_ms)
{
    uint32_t waited_ms = 0U;

    while (waited_ms < period_ms) {
        uint32_t step_ms = period_ms - waited_ms;

        if (step_ms > TASK_WATCH_STEP_MS) {
            step_ms = TASK_WATCH_STEP_MS;
        }
        osDelay(pdMS_TO_TICKS(step_ms));
        waited_ms += step_ms;
        if (task_stop_key()) return true;
    }
    return false;
}

/**
 * @description: 任务状态缩写成一个字符。
 * @param {eTaskState} state  FreeRTOS 任务状态
 * @return {char} R 运行，X 就绪，B 阻塞，S 挂起，D 删除
 */
static char task_state_char(eTaskState state)
{
    switch (state) {
    case eRunning:
        return 'R';
    case eReady:
        return 'X';
    case eBlocked:
        return 'B';
    case eSuspended:
        return 'S';
    case eDeleted:
        return 'D';
    default:
        return '?';
    }
}

/**
 * @description: 相对上一帧的运行计数增量。
 * @param {const TaskStatus_t *} prev  上一帧，可为空
 * @param {UBaseType_t} nprev  上一帧任务数
 * @param {UBaseType_t} number  任务序号
 * @param {uint32_t} now  当前运行计数
 * @return {uint32_t} 增量；上一帧没有该任务时为 0
 */
static uint32_t task_run_delta(const TaskStatus_t *prev, UBaseType_t nprev, UBaseType_t number, uint32_t now)
{
    UBaseType_t i;

    for (i = 0U; i < nprev; i++) {
        if (prev[i].xTaskNumber == number) return now - prev[i].ulRunTimeCounter;
    }
    return 0U;
}

/**
 * @description: 把一段运行计数换成十分之一百分点。
 * @param {uint32_t} part  任务计数
 * @param {uint32_t} total  全部任务计数
 * @return {uint32_t} 251 表示 25.1%
 */
static uint32_t task_percent_tenths(uint32_t part, uint32_t total)
{
    if (total == 0U) return 0U;
    return (uint32_t)(((uint64_t)part * 1000ULL) / (uint64_t)total);
}

/**
 * @description: 追加空格。
 * @param {textbuf_t *} tb  输出缓冲
 * @param {uint32_t} count  空格个数
 * @return {*} 无
 */
static void task_append_spaces(textbuf_t *tb, uint32_t count)
{
    uint32_t i;

    for (i = 0U; i < count; i++) {
        (void)textbuf_append_char(tb, ' ');
    }
}

/**
 * @description: 左对齐写入，不足补空格。
 * @param {textbuf_t *} tb  输出缓冲
 * @param {const char *} text  文本，可为空
 * @param {uint32_t} width  列宽，单位字符
 * @return {*} 无
 */
static void task_append_left(textbuf_t *tb, const char *text, uint32_t width)
{
    uint32_t i = 0U;

    if (text != NULL) {
        while ((text[i] != '\0') && (i < width)) {
            (void)textbuf_append_char(tb, text[i]);
            i++;
        }
    }
    if (i < width) {
        task_append_spaces(tb, width - i);
    }
}

/**
 * @description: 无符号数的十进制位数。
 * @param {uint32_t} value  数值
 * @return {uint32_t} 至少为 1
 */
static uint32_t task_u32_digits(uint32_t value)
{
    uint32_t digits = 1U;

    while (value >= 10U) {
        value /= 10U;
        digits++;
    }
    return digits;
}

/**
 * @description: 右对齐写入无符号数，不足补空格。
 * @param {textbuf_t *} tb  输出缓冲
 * @param {uint32_t} value  数值
 * @param {uint32_t} width  列宽，单位字符
 * @return {*} 无
 */
static void task_append_u32_right(textbuf_t *tb, uint32_t value, uint32_t width)
{
    uint32_t digits = task_u32_digits(value);

    if (digits < width) {
        task_append_spaces(tb, width - digits);
    }
    (void)textbuf_append_u32(tb, value);
}

/**
 * @description: 右对齐写入无符号数和可选后缀。
 * @param {textbuf_t *} tb  输出缓冲
 * @param {uint32_t} value  数值
 * @param {char} suffix  后缀；0 表示没有
 * @param {uint32_t} width  列宽，含后缀，单位字符
 * @return {*} 无
 */
static void task_append_u32_suffix(textbuf_t *tb, uint32_t value, char suffix, uint32_t width)
{
    uint32_t used = task_u32_digits(value);

    if (suffix != '\0') {
        used++;
    }
    if (used < width) {
        task_append_spaces(tb, width - used);
    }
    (void)textbuf_append_u32(tb, value);
    if (suffix != '\0') {
        (void)textbuf_append_char(tb, suffix);
    }
}

/**
 * @description: 右对齐写入一位小数的百分比，固定 6 字符。
 * @detail: 100.0%、 80.2%、  4.1% 的百分号落在同一列。无效时写 6 个空格宽的横线。
 * @param {textbuf_t *} tb  输出缓冲
 * @param {bool} valid  false=没有运行计数
 * @param {uint32_t} tenths  十分之一百分点
 * @return {*} 无
 */
static void task_append_cpu(textbuf_t *tb, bool valid, uint32_t tenths)
{
    uint32_t whole;
    uint32_t digits;

    if (!valid) {
        task_append_spaces(tb, TASK_COL_CPU - 1U);
        (void)textbuf_append_char(tb, '-');
        return;
    }
    whole = tenths / 10U;
    digits = task_u32_digits(whole);
    if (digits + 3U < TASK_COL_CPU) {
        task_append_spaces(tb, TASK_COL_CPU - (digits + 3U));
    }
    (void)textbuf_append_u32(tb, whole);
    (void)textbuf_append_char(tb, '.');
    (void)textbuf_append_u32(tb, tenths % 10U);
    (void)textbuf_append_char(tb, '%');
}

/**
 * @description: 在调度器已恢复后补每个任务的剩余栈。
 * @detail: 快照本身不再扫栈。每扫完一个任务至少让出 1 个 tick。
 *          mt6701 的采样超时是 2 ms，超时会锁存 ENCODER_SPI 并停机。
 * @param {TaskStatus_t *} tasks  快照，xHandle 必须有效
 * @param {UBaseType_t} count  任务数
 * @return {*} 无
 */
static void task_fill_stack(TaskStatus_t *tasks, UBaseType_t count)
{
    UBaseType_t i;

    for (i = 0U; i < count; i++) {
        tasks[i].usStackHighWaterMark = (configSTACK_DEPTH_TYPE)uxTaskGetStackHighWaterMark(tasks[i].xHandle);
        osDelay(pdMS_TO_TICKS(1U));
    }
}

/**
 * @description: 打印一帧任务、堆和 FOC 中断负载。
 * @detail: 中断耗时记在被打断的任务上，所以另给 FOC 负载。CPU 为 0 且总计数不变时，统计定时器没在走。
 * @param {const TaskStatus_t *} now  当前快照
 * @param {UBaseType_t} nnow  当前任务数
 * @param {const TaskStatus_t *} prev  上一帧；累计模式可为空
 * @param {UBaseType_t} nprev  上一帧任务数
 * @param {uint32_t} total_now  当前总运行计数
 * @param {uint32_t} total_prev  上一帧总运行计数
 * @param {bool} use_delta  true=本刷新间隔，false=上电累计
 * @return {*} 无
 */
static void task_print_frame(const TaskStatus_t *now, UBaseType_t nnow, const TaskStatus_t *prev, UBaseType_t nprev,
                             uint32_t total_now, uint32_t total_prev, bool use_delta)
{
    char line[96];
    textbuf_t tb;
    UBaseType_t order[TASK_STAT_SLOTS];
    uint32_t cpu[TASK_STAT_SLOTS];
    uint32_t total;
    UBaseType_t i;
    float foc_avg_us;
    float foc_adc_hz;
    uint32_t foc_load;

    total = use_delta ? (total_now - total_prev) : total_now;
    for (i = 0U; i < nnow; i++) {
        uint32_t part = use_delta ? task_run_delta(prev, nprev, now[i].xTaskNumber, now[i].ulRunTimeCounter)
                                  : now[i].ulRunTimeCounter;
        cpu[i] = part;
        order[i] = i;
    }
    for (i = 1U; i < nnow; i++) {
        UBaseType_t key = order[i];
        UBaseType_t j = i;

        while ((j > 0U) && (cpu[order[j - 1U]] < cpu[key])) {
            order[j] = order[j - 1U];
            j--;
        }
        order[j] = key;
    }

    foc_avg_us = Motor_DmaInt_GetAverageRuntimeUs();
    foc_adc_hz = Motor_DmaInt_GetAdcFrequencyHz();
    foc_load = 0U;
    if (foc_adc_hz > 0.0f) {
        foc_load = (uint32_t)((foc_avg_us * foc_adc_hz / 10000.0f) + 0.5f);
    }

    textbuf_init(&tb, line, sizeof(line));
    for (i = 0U; i < TASK_TABLE_WIDTH; i++) {
        (void)textbuf_append_char(&tb, '-');
    }
    (void)textbuf_append_str(&tb, "\r\n");
    task_emit(line, textbuf_length(&tb));

    textbuf_init(&tb, line, sizeof(line));
    task_append_left(&tb, "heap", 7U);
    task_append_left(&tb, "free", 8U);
    task_append_u32_suffix(&tb, (uint32_t)xPortGetFreeHeapSize(), '\0', 6U);
    (void)textbuf_append_char(&tb, ' ');
    task_append_left(&tb, "min", 8U);
    task_append_u32_suffix(&tb, (uint32_t)xPortGetMinimumEverFreeHeapSize(), '\0', 6U);
    (void)textbuf_append_char(&tb, ' ');
    task_append_left(&tb, "total", 8U);
    task_append_u32_suffix(&tb, (uint32_t)configTOTAL_HEAP_SIZE, '\0', 6U);
    (void)textbuf_append_str(&tb, "\r\n");
    task_emit(line, textbuf_length(&tb));

    textbuf_init(&tb, line, sizeof(line));
    task_append_left(&tb, "foc", 7U);
    task_append_left(&tb, "avg us", 8U);
    task_append_u32_suffix(&tb, (uint32_t)(foc_avg_us + 0.5f), '\0', 6U);
    (void)textbuf_append_char(&tb, ' ');
    task_append_left(&tb, "load", 8U);
    task_append_u32_suffix(&tb, foc_load, '%', 6U);
    (void)textbuf_append_char(&tb, ' ');
    task_append_left(&tb, "tasks", 8U);
    task_append_u32_suffix(&tb, (uint32_t)nnow, '\0', 6U);
    (void)textbuf_append_str(&tb, "\r\n");
    task_emit(line, textbuf_length(&tb));

    textbuf_init(&tb, line, sizeof(line));
    task_append_left(&tb, "name", TASK_NAME_WIDTH);
    (void)textbuf_append_char(&tb, ' ');
    task_append_left(&tb, "st", TASK_COL_STATE);
    (void)textbuf_append_char(&tb, ' ');
    task_append_left(&tb, "pri", TASK_COL_PRI);
    (void)textbuf_append_char(&tb, ' ');
    task_append_left(&tb, "free_w", TASK_COL_STACK);
    (void)textbuf_append_char(&tb, ' ');
    task_append_spaces(&tb, TASK_COL_CPU - 4U);
    (void)textbuf_append_str(&tb, "cpu%\r\n");
    task_emit(line, textbuf_length(&tb));

    for (i = 0U; i < nnow; i++) {
        const TaskStatus_t *task = &now[order[i]];
        char state_text[2];

        state_text[0] = task_state_char(task->eCurrentState);
        state_text[1] = '\0';
        textbuf_init(&tb, line, sizeof(line));
        task_append_left(&tb, task->pcTaskName, TASK_NAME_WIDTH);
        (void)textbuf_append_char(&tb, ' ');
        task_append_left(&tb, state_text, TASK_COL_STATE);
        (void)textbuf_append_char(&tb, ' ');
        task_append_u32_right(&tb, (uint32_t)task->uxCurrentPriority, TASK_COL_PRI);
        (void)textbuf_append_char(&tb, ' ');
        task_append_u32_right(&tb, (uint32_t)task->usStackHighWaterMark, TASK_COL_STACK);
        (void)textbuf_append_char(&tb, ' ');
        task_append_cpu(&tb, total != 0U, task_percent_tenths(cpu[order[i]], total));
        (void)textbuf_append_str(&tb, "\r\n");
        task_emit(line, textbuf_length(&tb));
    }
}

/**
 * @description: 按周期刷新任务占用，直到串口收到任意输入。
 * @param {uint32_t} period_ms  刷新周期，单位 ms
 * @return {*} 无
 */
static void task_watch(uint32_t period_ms)
{
    char line[96];
    textbuf_t tb;
    UBaseType_t nprev;
    uint32_t total_prev = 0U;

    textbuf_init(&tb, line, sizeof(line));
    (void)textbuf_append_str(&tb, "task watch ");
    (void)textbuf_append_u32(&tb, period_ms);
    (void)textbuf_append_str(&tb, " ms, any key stops\r\n");
    task_emit(line, textbuf_length(&tb));
    task_emit_str("CPU 含被打断任务里的中断时间，FOC 负载单独给出\r\n");

    nprev = uxTaskGetSystemState(s_task_prev, TASK_STAT_SLOTS, &total_prev);
    if ((nprev == 0U) || (nprev > TASK_STAT_SLOTS)) {
        task_emit_str("task: too many tasks\r\n");
        return;
    }

    while (task_wait_or_key(period_ms) == false) {
        uint32_t total_now = 0U;
        UBaseType_t nnow = uxTaskGetSystemState(s_task_now, TASK_STAT_SLOTS, &total_now);

        if ((nnow == 0U) || (nnow > TASK_STAT_SLOTS)) {
            task_emit_str("task: too many tasks\r\n");
            break;
        }
        task_fill_stack(s_task_now, nnow);
        task_print_frame(s_task_now, nnow, s_task_prev, nprev, total_now, total_prev, true);
        memcpy(s_task_prev, s_task_now, sizeof(s_task_now[0]) * nnow);
        nprev = nnow;
        total_prev = total_now;
    }
    task_emit_str("task watch stopped\r\n");
}

/**
 * @description: 打印 FreeRTOS 任务占用、剩余栈和堆。
 * @detail: `task` 打上电累计；`task watch [ms]` 按间隔刷新，任意键停止。周期限制在 200~5000 ms。
 *          任务表在调度锁里只拷贝状态，剩余栈在锁外逐个任务扫描，避免卡住速度环。
 *          发送走队列整段写入，不按字节关中断。
 * @param {int} argc  参数个数
 * @param {char *[]} argv  参数；argv[1] 为 watch 时进入刷新
 * @return {int} 0
 * @author: lkc
 */
int Cmd_Task_Show(int argc, char *argv[])
{
    UBaseType_t nnow;
    uint32_t total_now = 0U;

    if ((argc >= 2) && (strcmp(argv[1], "watch") == 0)) {
        uint32_t period_ms = 500U;

        if (argc >= 3) {
            period_ms = (uint32_t)atoi(argv[2]);
        }
        if (period_ms < TASK_WATCH_MIN_MS) {
            period_ms = TASK_WATCH_MIN_MS;
        }
        if (period_ms > TASK_WATCH_MAX_MS) {
            period_ms = TASK_WATCH_MAX_MS;
        }
        task_watch(period_ms);
        return 0;
    }
    if (argc >= 2) {
        task_emit_str("usage: task [watch [ms]]\r\n");
        return 0;
    }

    nnow = uxTaskGetSystemState(s_task_now, TASK_STAT_SLOTS, &total_now);
    if ((nnow == 0U) || (nnow > TASK_STAT_SLOTS)) {
        task_emit_str("task: too many tasks\r\n");
        return 0;
    }
    task_fill_stack(s_task_now, nnow);
    task_emit_str("CPU 含被打断任务里的中断时间，FOC 负载单独给出\r\n");
    task_print_frame(s_task_now, nnow, NULL, 0U, total_now, 0U, false);
    return 0;
}

#ifdef __cplusplus
}
#endif
