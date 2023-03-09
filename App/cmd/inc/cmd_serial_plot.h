/*
 * @Description: 
 * @Date: 2023-02-18 23:29:45
 * @LastEditTime: 2023-03-04 18:58:19
 * @FilePath: \foc\Library\Bsp\inc\bsp_adc.h
 */
#ifndef _COMMAND_SERIAL_PLOT_H
#define _COMMAND_SERIAL_PLOT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TEXT_PLOT(title, fmt, args...) printf("{"#title"}"fmt"\n", ##args)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
