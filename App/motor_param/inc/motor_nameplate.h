/*
 * @Description:
 * @Version: 1.0
 * @Author: lkc
 * @detail:
 */
/*
 * @Description: 当前电机型号选择入口
 * @Author: lkc
 */
#ifndef MOTOR_NAMEPLATE_H
#define MOTOR_NAMEPLATE_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_MODEL_ATK_BL57H95D20E (1U)
#define MOTOR_MODEL_C2208_100T      (2U)

/*
 * 在工程设置中定义 MOTOR_SELECTED_MODEL 可覆盖默认选择；也可在此处修改默认型号。
 * 当前默认保持 ATK-BL57H95D20E，避免改变已有控制参数和上板基线。
 */
#ifndef MOTOR_SELECTED_MODEL
#define MOTOR_SELECTED_MODEL MOTOR_MODEL_C2208_100T
#endif

#if MOTOR_SELECTED_MODEL == MOTOR_MODEL_ATK_BL57H95D20E
#include "motor_atk_bl57h95d20e.h"
#elif MOTOR_SELECTED_MODEL == MOTOR_MODEL_C2208_100T
#include "motor_c2208_100t.h"
#else
#error "Unsupported MOTOR_SELECTED_MODEL"
#endif

#if !defined(MOTOR_FIXED_MODEL) || !defined(MOTOR_FIXED_POLE_PAIRS) || !defined(MOTOR_FIXED_RATED_RPM) ||       \
    !defined(MOTOR_FIXED_LINE_RESISTANCE) || !defined(MOTOR_FIXED_LINE_INDUCTANCE) ||                          \
    !defined(MOTOR_FIXED_FLUX_LINKAGE)
#error "Selected motor header does not provide the required MOTOR_FIXED_* parameters"
#endif

/* 旧的自定义电机头未提供补偿参数时保持关闭，避免改变原行为。 */
#ifndef MOTOR_ENCODER_START_IQ_A
#define MOTOR_ENCODER_START_IQ_A (0.0f)
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
