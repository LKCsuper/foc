/*
 * @Description: 检测参数
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-10-03 19:43:57
 * @FilePath: \foc\App\motor\inc\utils.h
 */
#ifndef _MOTOR_DETECT_H
#define _MOTOR_DETECT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
ULONG DETECT_Offset(BOOL cal_undriven);
VOID Detect_Init(VOID);
VOID DETECT_Hall(VOID);
/*
 * 查询偏移检测是否正在运行。
 * 返回 true 时，检测代码正在独占 TIM1/PWM；FOC 中断必须立即退出，
 * 但 ADC-DMA 硬件仍会继续把最新采样值写入 ADC_Value[]。
 */
bool Motor_Detect_IsActive(void);
#ifdef __cplusplus
}
#endif

#endif
