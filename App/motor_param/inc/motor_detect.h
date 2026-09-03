/*
 * @Description: 电流/电压偏移校准
 * @Author: lkc
 */
#ifndef _MOTOR_DETECT_H
#define _MOTOR_DETECT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_DETECT_RESULT_VERSION (1U)
/* Private typedef -----------------------------------------------------------*/
/*
 * Flash 槽里仍保存该结构，布局不能改。R/L/磁链检测已停用，上电不再应用这些字段。
 */
typedef enum {
    MOTOR_DETECT_ERROR_NONE = 0,
    MOTOR_DETECT_ERROR_BUSY,
    MOTOR_DETECT_ERROR_MOTOR_RUNNING,
    MOTOR_DETECT_ERROR_FAULT,
    MOTOR_DETECT_ERROR_TIMEOUT,
    MOTOR_DETECT_ERROR_CURRENT,
    MOTOR_DETECT_ERROR_RESISTANCE,
    MOTOR_DETECT_ERROR_INDUCTANCE,
    MOTOR_DETECT_ERROR_FLUX,
    MOTOR_DETECT_ERROR_MECHANICAL_REFERENCE,
    MOTOR_DETECT_ERROR_USER_ABORT,
} motor_detect_error_t;

typedef struct {
    uint32_t version;
    bool resistance_valid;
    bool inductance_valid;
    bool flux_valid;
    bool pole_pairs_valid;
    float resistance_ohm;
    float inductance_h;
    float flux_linkage_wb;
    float pole_pairs;
    float test_current_a;
    float test_electrical_rpm;
    motor_detect_error_t error;
    mc_fault_code fault;
} motor_detect_result_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 校准三相电流零点和驱动时相电压偏移
 * @param {BOOL} cal_undriven  true 时额外校准未驱动相电压偏移
 * @return {ULONG} 1 成功，0 被中止
 * @author: lkc
 */
ULONG DETECT_Offset(BOOL cal_undriven);

/**
 * @description: 上电做一次电流/电压偏移校准
 * @return {*}
 * @author: lkc
 */
VOID Detect_Init(VOID);

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
 * @return {*}
 * @author: lkc
 */
VOID DETECT_Hall(VOID);

/**
 * @description: 偏移校准时 FOC 中断必须立刻返回，避免和任务抢写 PWM
 * @return {bool} true 表示校准占用功率桥
 * @author: lkc
 */
bool Motor_Detect_IsActive(void);

#ifdef __cplusplus
}
#endif
#endif
