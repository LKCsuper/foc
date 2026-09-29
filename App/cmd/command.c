/*
 * @Description: 精简命令行框架
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2025-07-13
 * @detail:
 *   命令集（39 条）全部汇总在这一张静态表里，不带 linker section 魔法。
 *   新增命令时在这里加一行 extern + 表项即可。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CMD_BUF_SIZE        160U        /* 单行输入缓冲 */
#define CMD_MAX_ARGS        8U          /* 最多参数个数（含命令名） */
#define CMD_HISTORY_COUNT   8U          /* 最近执行的命令条数 */
/* Private macro -------------------------------------------------------------*/
/* -- 前向声明：所有命令函数统一签名 int func(int argc, char *argv[]) ----------*/

/* cmd_detect.c       */ extern int Cmd_Detect(int argc, char *argv[]);
/* cmd_env.c          */ extern int Cmd_Saveparam(int argc, char *argv[]);
/* cmd_misc.c         */ extern int CMD_Reset(int argc, char *argv[]);
/* cmd_show_param.c   */ extern int Cmd_Show(int argc, char *argv[]);
/* cmd_task.c         */ extern int Cmd_Task_Show(int argc, char *argv[]);
/* cmd_duty.c         */ extern int Cmd_Motor_Duty(int argc, char *argv[]);
/* cmd_encoder.c      */ extern int Cmd_Motor_Encoder(int argc, char *argv[]);
/* cmd_serial_plot.c  */ extern int Cmd_Serial_Plot(int argc, char *argv[]);
/* cmd_bldc.c         */ extern int Cmd_Bldc(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_Motor_OpenLoop(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlSpeed(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlStatus(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlSnap(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlCurrentDiag(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlBoost(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlKvf(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlRamp(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlVlimit(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlMaxSpeed(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlMinSpeed(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlStart(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlStop(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlFault(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlMode(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlFeedback(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlIq(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlId(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_Fw(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlKp(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlKi(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlPhase(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_OlAlign(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_HfiStart(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SpeedStatus(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SpeedEnable(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SpeedKp(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SpeedKi(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SpeedIqLimit(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SpeedBrake(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SensorlessEnable(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SensorlessTakeover(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SensorlessRpm(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SensorlessGuard(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SensorlessMotor(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SensorlessObserver(int argc, char *argv[]);
/* cmd_openloop.c     */ extern int Cmd_SensorlessPhase(int argc, char *argv[]);

/* -- 命令表 ----------------------------------------------------------------*/
static const cmd_entry_t s_cmd_table[] = {
    /* 偏移校准 / 霍尔检测 */
    { "detect",     Cmd_Detect,             "detect offset/hall/phase" },
    { "bldc",       Cmd_Bldc,               "BLDC: start/open | stop | exit | status" },
    /* Flash 存储 */
    { "saveparam",  Cmd_Saveparam,          "save motor params to Flash" },
    /* 开环 V/F 控制 */
    { "olspeed",    Cmd_OlSpeed,            "openloop V/F speed set <rpm>" },
    { "olstatus",   Cmd_OlStatus,           "show openloop current and fault status" },
    { "olsnap",     Cmd_OlSnap,             "short Id/Iq/RPM/fault snapshot while running" },
    { "olcurdiag",  Cmd_OlCurrentDiag,      "Hall current errors by sector since last start" },
    { "olboost",    Cmd_OlBoost,            "openloop V/F boost voltage <V>" },
    { "olkvf",      Cmd_OlKvf,              "openloop V/F slope <V/Hz>" },
    { "olramp",     Cmd_OlRamp,             "openloop V/F ramp rate <rpm/s>" },
    { "olvlimit",   Cmd_OlVlimit,           "openloop Vq limit <V>" },
    { "olmax",      Cmd_OlMaxSpeed,         "openloop max speed <rpm>" },
    { "olmin",      Cmd_OlMinSpeed,         "openloop min speed <rpm>" },
    { "olstart",    Cmd_OlStart,            "openloop start" },
    { "olstop",     Cmd_OlStop,             "openloop stop" },
    { "olfault",    Cmd_OlFault,            "show or clear latched motor fault" },
    { "olmode",     Cmd_OlMode,             "openloop mode 0=VF 1=current" },
    { "olfb",       Cmd_OlFeedback,         "feedback openloop|sensorless|encoder|hall" },
    { "oliq",       Cmd_OlIq,               "openloop current Iq target <A>" },
    { "olid",       Cmd_OlId,               "openloop current Id target <A>" },
    { "fw",         Cmd_Fw,                 "field weakening: <max_A> [duty_start] [ramp_A/s]" },
    { "olkp",       Cmd_OlKp,               "openloop current Kp" },
    { "olki",       Cmd_OlKi,               "openloop current Ki" },
    { "olphase",    Cmd_OlPhase,            "openloop start phase boost <deg>" },
    { "olalign",    Cmd_OlAlign,            "startup rotor alignment config" },
    { "hfistart",   Cmd_HfiStart,           "HFI startup: <0|1> <V> <cycles> [confidence]" },
    /* 速度环 */
    { "speed",      Cmd_SpeedStatus,        "show speed loop status" },
    { "spenable",   Cmd_SpeedEnable,        "enable speed PI loop <0/1>" },
    { "spkp",       Cmd_SpeedKp,            "set speed PI Kp" },
    { "spki",       Cmd_SpeedKi,            "set speed PI Ki" },
    { "spiq",       Cmd_SpeedIqLimit,       "set speed PI Iq limit <A>" },
    { "spbrake",    Cmd_SpeedBrake,         "allow speed loop braking <0/1>" },
    /* 无感观测器 */
    { "slenable",   Cmd_SensorlessEnable,   "enable sensorless observer <0/1>" },
    { "sltakeover", Cmd_SensorlessTakeover, "allow observer angle takeover <0/1>" },
    { "slrpm",      Cmd_SensorlessRpm,      "set sensorless transition rpm" },
    { "slguard",    Cmd_SensorlessGuard,    "set I/F ceiling and takeover guard" },
    { "slmotor",    Cmd_SensorlessMotor,    "set sensorless motor model" },
    { "slobs",      Cmd_SensorlessObserver, "set observer and PLL gains" },
    { "slphase",    Cmd_SensorlessPhase,    "set observer phase offset <deg>" },
    /* 杂项 */
    { "show",       Cmd_Show,               "show hall sensor state" },
    { "task",       Cmd_Task_Show,          "show FreeRTOS task stats" },
    { "reset",      CMD_Reset,              "software reset MCU" },
    { "duty",       Cmd_Motor_Duty,         "duty cycle control: create/destory/set/start/stop" },
    { "encoder",    Cmd_Motor_Encoder,      "AB encoder show/reset/detect" },
    { "plot",       Cmd_Serial_Plot,        "VOFA JustFloat: create current|speed|pos|sine" },
    { "openloop",   Cmd_Motor_OpenLoop,     "openloop background task create" },
};

static const unsigned int s_cmd_count = sizeof(s_cmd_table) / sizeof(s_cmd_table[0]);

/* -- 命令任务 ---------------------------------------------------------------*/
static StackType_t  taskCmdStack[2048];
static StaticTask_t taskCmdTcb;
const osThreadAttr_t taskCmdAttr = {
    .name       = "Task Cmd",
    .priority   = PRI_CMD,
    .cb_mem     = &taskCmdTcb,
    .cb_size    = sizeof(taskCmdTcb),
    .stack_mem  = &taskCmdStack,
    .stack_size = sizeof(taskCmdStack),
};

QueueHandle_t queueCmdHandle;
#define CMD_QUEUE_LENGTH    8
#define CMD_QUEUE_SIZE      sizeof(CMD_QUEUE_ITEM)

/* -- 解析 & 分发 ------------------------------------------------------------*/
static void cmd_dispatch(char *line)
{
    int   argc = 0;
    char *argv[CMD_MAX_ARGS];
    char *token;
    unsigned int i;

    token = strtok(line, " \t\r\n");
    while (token != NULL && argc < (int)CMD_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }
    if (argc == 0) return;

    for (i = 0; i < s_cmd_count; i++) {
        if (strcmp(argv[0], s_cmd_table[i].name) == 0) {
            s_cmd_table[i].func(argc, argv);
            return;
        }
    }

    printf("unknown: %s (try 'help')\r\n", argv[0]);
}

/* -- 回显 & 编辑支持 --------------------------------------------------------*/
static char     s_cmd_buf[CMD_BUF_SIZE];
static uint8_t  s_cmd_len;
static char     s_cmd_history[CMD_HISTORY_COUNT][CMD_BUF_SIZE];
static char     s_cmd_draft[CMD_BUF_SIZE];
static uint8_t  s_cmd_history_head;
static uint8_t  s_cmd_history_count;
static uint8_t  s_cmd_history_depth;
static uint8_t  s_cmd_draft_len;
static uint8_t  s_cmd_escape_state;

/**
 * @description: 把上电以来的时基格式化为 `[T+秒.毫秒s]`。
 * @param {char *} buf 输出缓冲，不可为空
 * @param {unsigned} buf_size 缓冲字节数
 * @return {*}
 */
void Cmd_FormatUptime(char *buf, unsigned buf_size)
{
    uint32_t us;
    unsigned sec;
    unsigned ms;

    if ((buf == NULL) || (buf_size == 0U)) {
        return;
    }

    us = Hal_Motor_TimeUs();
    sec = (unsigned)(us / 1000000UL);
    ms = (unsigned)((us / 1000UL) % 1000UL);
    (void)snprintf(buf, buf_size, "[T+%u.%03us]", sec, ms);
}

static void cmd_putc(char c)
{
    Bsp_Usart_PutChar((uint8_t)c);
}

static void cmd_put_prompt(void)
{
    char ts[20];

    Cmd_FormatUptime(ts, sizeof(ts));
    printf("%s $ ", ts);
}

static void cmd_print_run_stamp(const char *line)
{
    char ts[20];

    Cmd_FormatUptime(ts, sizeof(ts));
    printf("%s > %s\r\n", ts, line);
}

static void cmd_replace_line(const char *line)
{
    while (s_cmd_len > 0U) {
        cmd_putc('\b');
        cmd_putc(' ');
        cmd_putc('\b');
        s_cmd_len--;
    }

    while ((line[s_cmd_len] != '\0') && (s_cmd_len < (CMD_BUF_SIZE - 1U))) {
        s_cmd_buf[s_cmd_len] = line[s_cmd_len];
        cmd_putc(s_cmd_buf[s_cmd_len]);
        s_cmd_len++;
    }
    s_cmd_buf[s_cmd_len] = '\0';
}

static void cmd_history_add(const char *line)
{
    uint8_t latest;

    if (s_cmd_history_count > 0U) {
        latest = (uint8_t)((s_cmd_history_head + CMD_HISTORY_COUNT - 1U) % CMD_HISTORY_COUNT);
        if (strcmp(line, s_cmd_history[latest]) == 0) {
            return;
        }
    }

    strncpy(s_cmd_history[s_cmd_history_head], line, CMD_BUF_SIZE - 1U);
    s_cmd_history[s_cmd_history_head][CMD_BUF_SIZE - 1U] = '\0';
    s_cmd_history_head = (uint8_t)((s_cmd_history_head + 1U) % CMD_HISTORY_COUNT);
    if (s_cmd_history_count < CMD_HISTORY_COUNT) {
        s_cmd_history_count++;
    }
}

static void cmd_history_up(void)
{
    uint8_t index;

    if ((s_cmd_history_count == 0U) || (s_cmd_history_depth >= s_cmd_history_count)) {
        return;
    }

    if (s_cmd_history_depth == 0U) {
        memcpy(s_cmd_draft, s_cmd_buf, s_cmd_len);
        s_cmd_draft[s_cmd_len] = '\0';
        s_cmd_draft_len = s_cmd_len;
    }

    s_cmd_history_depth++;
    index = (uint8_t)((s_cmd_history_head + CMD_HISTORY_COUNT - s_cmd_history_depth) % CMD_HISTORY_COUNT);
    cmd_replace_line(s_cmd_history[index]);
}

static void cmd_history_down(void)
{
    uint8_t index;

    if (s_cmd_history_depth == 0U) {
        return;
    }

    s_cmd_history_depth--;
    if (s_cmd_history_depth == 0U) {
        s_cmd_draft[s_cmd_draft_len] = '\0';
        cmd_replace_line(s_cmd_draft);
        return;
    }

    index = (uint8_t)((s_cmd_history_head + CMD_HISTORY_COUNT - s_cmd_history_depth) % CMD_HISTORY_COUNT);
    cmd_replace_line(s_cmd_history[index]);
}

static void cmd_handle_char(char c)
{
    if (s_cmd_escape_state == 1U) {
        s_cmd_escape_state = (c == '[') ? 2U : 0U;
        return;
    }
    if (s_cmd_escape_state == 2U) {
        if (c == 'A') {
            cmd_history_up();
        } else if (c == 'B') {
            cmd_history_down();
        }
        s_cmd_escape_state = 0U;
        return;
    }
    if ((uint8_t)c == 0x1BU) {
        s_cmd_escape_state = 1U;
        return;
    }

    if (c == '\r' || c == '\n') {
        cmd_putc('\r');
        cmd_putc('\n');
        if (s_cmd_len > 0) {
            s_cmd_buf[s_cmd_len] = '\0';
            cmd_history_add(s_cmd_buf);
            cmd_print_run_stamp(s_cmd_buf);
            if (strcmp(s_cmd_buf, "help") == 0) {
                unsigned int i;
                printf("Commands:\r\n");
                for (i = 0; i < s_cmd_count; i++) {
                    printf("  %-12s - %s\r\n", s_cmd_table[i].name, s_cmd_table[i].desc);
                }
            } else {
                cmd_dispatch(s_cmd_buf);
            }
            s_cmd_len = 0;
        }
        s_cmd_history_depth = 0U;
        s_cmd_draft_len = 0U;
        cmd_put_prompt();
    } else if (c == '\b' || c == 0x7F) {
        if (s_cmd_len > 0) {
            s_cmd_history_depth = 0U;
            s_cmd_len--;
            cmd_putc('\b');
            cmd_putc(' ');
            cmd_putc('\b');
        }
    } else if (c >= ' ' && s_cmd_len < CMD_BUF_SIZE - 1) {
        s_cmd_history_depth = 0U;
        s_cmd_buf[s_cmd_len++] = c;
        cmd_putc(c);
    }
}

/**
 * @description: 命令行任务 — 从队列读取串口字符并解析分发命令。
 * @param {void *} argument 任务参数（未使用）
 * @return {void} 无
 * @author: lkc
 */
void Cmd_Task(void *argument)
{
    STATIC CMD_QUEUE_ITEM rx_item;
    ULONG i;

    (void)argument;
    printf("Cmd Task Init\r\n");
    printf("Type 'help' for command list\r\n");
    cmd_put_prompt();

    while (1) {
        if (osMessageQueueGet(queueCmdHandle, &rx_item, NULL, pdMS_TO_TICKS(1)) == osOK) {
            for (i = 0; i < rx_item.len; i++) {
                cmd_handle_char((char)rx_item.data[i]);
            }
        }
        /* FOC 高负载时 UART TX DMA 完成中断经常进不来；空闲 1ms 补一轮，把回绕后的下一段发出去。 */
        Bsp_Usart_TxDmaPoll();
    }
}

/* -- 初始化 ----------------------------------------------------------------*/
/**
 * @description: 初始化命令行框架 — 创建消息队列与命令任务。
 * @return {void} 无
 * @author: lkc
 */
void Command_Init(void)
{
    if (queueCmdHandle == NULL) {
        queueCmdHandle = osMessageQueueNew(CMD_QUEUE_LENGTH, CMD_QUEUE_SIZE, NULL);
    }

    osThreadNew(Cmd_Task, NULL, &taskCmdAttr);
}

#ifdef __cplusplus
}
#endif
