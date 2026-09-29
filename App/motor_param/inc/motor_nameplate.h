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
 * 当前上板调试默认选择 C2208-100T；切换 ATK 时只需选择对应电机头。
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
#ifndef MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM
#define MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM (0.0f)
#endif
#ifndef MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM
#define MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM (0.0f)
#endif
#ifndef MOTOR_ENCODER_SPEED_VOLTAGE_MODE
#define MOTOR_ENCODER_SPEED_VOLTAGE_MODE (0U)
#endif
#ifndef MOTOR_ENCODER_SPEED_VOLT_PER_AMP
#define MOTOR_ENCODER_SPEED_VOLT_PER_AMP (0.0f)
#endif
#ifndef MOTOR_ENCODER_SPEED_VOLT_LIMIT_V
#define MOTOR_ENCODER_SPEED_VOLT_LIMIT_V (0.0f)
#endif
#ifndef MOTOR_ENCODER_SPEED_FF_BOOST_V
#define MOTOR_ENCODER_SPEED_FF_BOOST_V (0.0f)
#endif
#ifndef MOTOR_ENCODER_SPEED_FF_V_PER_RPM
#define MOTOR_ENCODER_SPEED_FF_V_PER_RPM (0.0f)
#endif
#ifndef MOTOR_ENCODER_PLL_BW_RAD_S
#define MOTOR_ENCODER_PLL_BW_RAD_S (100.0f)
#endif
#ifndef MOTOR_SPEED_KP_DEFAULT
#define MOTOR_SPEED_KP_DEFAULT (0.003f)
#endif
#ifndef MOTOR_SPEED_KI_DEFAULT
#define MOTOR_SPEED_KI_DEFAULT (0.01f)
#endif
#ifndef MOTOR_SPEED_IQ_LIMIT_DEFAULT
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT (2.0f)
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
