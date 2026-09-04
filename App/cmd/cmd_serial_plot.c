/*
 * @Description: VOFA+ 串口绘图（JustFloat）命令
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum {
    PLOT_MODE_NONE = 0,
    PLOT_MODE_CURRENT, /* ia ib ic v_bus */
    PLOT_MODE_SPEED,   /* 目标/实测转速、误差、Iq */
    PLOT_MODE_POS,     /* 机械角/电角/开环角/Iq */
    PLOT_MODE_SINE,    /* θ、sinθ、cosθ、Id、Iq */
} plot_mode_t;
/* Private define ------------------------------------------------------------*/
#define PLOT_PERIOD_MS          (5U)   /* 约 200 Hz，适配 460800 波特 */
#define PLOT_MAX_CH             (5U)
/* JustFloat 帧尾：00 00 80 7f */
static const uint8_t s_justfloat_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TaskHandle_t s_plot_task;
static volatile plot_mode_t s_plot_mode = PLOT_MODE_NONE;

static const osThreadAttr_t s_plot_attr = {
    .name = "Task Plot",
    .priority = PRI_INIT,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 以 VOFA+ JustFloat 发送一帧浮点通道
 * @param {float} *ch  通道数组
 * @param {uint32_t} n 通道数
 * @return {*}
 * @author: lkc
 */
static void Vofa_SendJustFloat(const float *ch, uint32_t n)
{
    if (ch == NULL || n == 0U || n > PLOT_MAX_CH) {
        return;
    }
    Bsp_Usart_PutBuf((const UCHAR *)ch, n * (ULONG)sizeof(float));
    Bsp_Usart_PutBuf(s_justfloat_tail, 4U);
}

/**
 * @description: 按当前模式采集通道并发送
 * @return {*}
 * @author: lkc
 */
static void Plot_SendFrame(void)
{
    float ch[PLOT_MAX_CH];
    float theta;
    uint32_t n = 0U;

    switch (s_plot_mode) {
    case PLOT_MODE_CURRENT:
        ch[0] = motor.ia;
        ch[1] = motor.ib;
        ch[2] = motor.ic;
        ch[3] = motor.v_bus;
        n = 4U;
        break;
    case PLOT_MODE_SPEED:
        ch[0] = motor.ol_target_rpm;
        ch[1] = Motor_SpeedLoop_GetMeasuredRpm();
        ch[2] = motor.ol_current_rpm;
        ch[3] = Motor_SpeedLoop_GetError();
        ch[4] = motor.speed_enable ? Motor_SpeedLoop_GetIqTarget() : motor.ol_iq_set;
        n = 5U;
        break;
    case PLOT_MODE_POS:
        ch[0] = Motor_Encoder_GetMechAngle();
        ch[1] = Motor_Encoder_GetElecAngle();
        ch[2] = motor.ol_theta;
        ch[3] = motor.Iq;
        ch[4] = motor.Id;
        n = 5U;
        break;
    case PLOT_MODE_SINE:
        theta = motor.ol_theta;
        ch[0] = theta;
        ch[1] = sinf(theta);
        ch[2] = cosf(theta);
        ch[3] = motor.Id;
        ch[4] = motor.Iq;
        n = 5U;
        break;
    default:
        return;
    }

    Vofa_SendJustFloat(ch, n);
}

/**
 * @description: 串口绘图任务，周期发送 JustFloat 帧
 * @param {void} *argument  未使用
 * @return {*}
 * @author: lkc
 */
static VOID Serial_PlotTask(VOID *argument)
{
    (void)argument;

    while (1) {
        if (s_plot_mode != PLOT_MODE_NONE) {
            Plot_SendFrame();
        }
        osDelay(pdMS_TO_TICKS(PLOT_PERIOD_MS));
    }
}

/**
 * @description: 打印用法与通道说明
 * @return {*}
 * @author: lkc
 */
static void Plot_PrintHelp(void)
{
    PRINTF("usage:\r\n");
    PRINTF("  plot create current|speed|pos|sine\r\n");
    PRINTF("  plot destory\r\n");
    PRINTF("VOFA+: protocol=JustFloat, baud=%u\r\n", (unsigned)DEBUG_USART_BAUD);
    PRINTF("channels:\r\n");
    PRINTF("  current: ia,ib,ic,v_bus\r\n");
    PRINTF("  speed:   tgt_rpm,meas_rpm,ramp_rpm,err_rpm,iq_ref\r\n");
    PRINTF("  pos:     mech_rad,elec_rad,ol_theta,Iq,Id\r\n");
    PRINTF("  sine:    theta,sin,cos,Id,Iq\r\n");
}

/**
 * @description: 解析绘图模式名
 * @param {char} *name  模式字符串
 * @return {plot_mode_t}
 * @author: lkc
 */
static plot_mode_t Plot_ParseMode(const char *name)
{
    if (strcmp(name, "current") == 0) {
        return PLOT_MODE_CURRENT;
    }
    if (strcmp(name, "speed") == 0) {
        return PLOT_MODE_SPEED;
    }
    if (strcmp(name, "pos") == 0) {
        return PLOT_MODE_POS;
    }
    if (strcmp(name, "sine") == 0 || strcmp(name, "sin") == 0) {
        return PLOT_MODE_SINE;
    }
    return PLOT_MODE_NONE;
}

/**
 * @description: plot 命令 — VOFA+ JustFloat 调试曲线
 * @param {int} argc
 * @param {char *} argv[]
 * @return {int} 0 成功，-1 失败
 * @author: lkc
 */
int Cmd_Serial_Plot(int argc, char *argv[])
{
    plot_mode_t mode;

    if (argc < 2) {
        Plot_PrintHelp();
        return -1;
    }

    if (strcmp(argv[1], "create") == 0) {
        if (argc < 3) {
            Plot_PrintHelp();
            return -1;
        }
        mode = Plot_ParseMode(argv[2]);
        if (mode == PLOT_MODE_NONE) {
            PRINTF("unknown mode: %s\r\n", argv[2]);
            Plot_PrintHelp();
            return -1;
        }

        s_plot_mode = mode;
        if (s_plot_task == NULL) {
            s_plot_task = osThreadNew(Serial_PlotTask, NULL, &s_plot_attr);
            if (s_plot_task == NULL) {
                s_plot_mode = PLOT_MODE_NONE;
                PRINTF("plot: create task failed\r\n");
                return -1;
            }
        }
        PRINTF("plot: JustFloat mode=%s @%ums\r\n", argv[2], (unsigned)PLOT_PERIOD_MS);
        Plot_PrintHelp();
        return 0;
    }

    if (strcmp(argv[1], "destory") == 0 || strcmp(argv[1], "destroy") == 0 ||
        strcmp(argv[1], "stop") == 0) {
        s_plot_mode = PLOT_MODE_NONE;
        if (s_plot_task != NULL) {
            if (osThreadTerminate(s_plot_task) == osOK) {
                s_plot_task = NULL;
                PRINTF("plot: stopped\r\n");
            } else {
                PRINTF("plot: stop err\r\n");
                return -1;
            }
        } else {
            PRINTF("plot: not running\r\n");
        }
        return 0;
    }

    if (strcmp(argv[1], "help") == 0) {
        Plot_PrintHelp();
        return 0;
    }

    PRINTF("unknown: %s\r\n", argv[1]);
    Plot_PrintHelp();
    return -1;
}

#ifdef __cplusplus
}
#endif
