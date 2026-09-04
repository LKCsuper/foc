/*
 * @Description: VOFA+ 串口绘图命令声明
 * @Author: lkc
 */
#ifndef _COMMAND_SERIAL_PLOT_H
#define _COMMAND_SERIAL_PLOT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* FireWater 文本格式（可选，主路径用 JustFloat） */
#define TEXT_PLOT(title, fmt, args...) printf("{" #title "}" fmt "\n", ##args)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: plot 命令 — VOFA+ JustFloat 调试曲线
 * @param {int} argc  参数个数
 * @param {char *} argv[]  create/destory/help 与模式名
 * @return {int} 0 成功，-1 失败
 * @author: lkc
 */
int Cmd_Serial_Plot(int argc, char *argv[]);

#ifdef __cplusplus
}
#endif

#endif
