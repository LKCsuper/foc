/*
 * @Description: 工程总入口头文件（平台 + 应用）
 * @Author: lkc
 * @detail: 业务 .c 仍可只 #include "includes.h"。拆分见 includes_platform.h /
 *          includes_app.h；勿重排本文件 include 顺序。
 */
#ifndef _INCLUDES_H
#define _INCLUDES_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes_platform.h"
#include "includes_app.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
