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

#define MOTOR_DETECT_RESULT_VERSION  (1U)

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

/* 自动检测相电阻、平均相电感和磁链；磁链阶段会让电机开环旋转。 */
bool DETECT_MotorParameters(float current_a, float electrical_rpm,
                            motor_detect_result_t *result);

/* 使用电气转速和外部测得的机械转速计算极对数。 */
bool DETECT_CalculatePolePairs(float electrical_rpm, float mechanical_rpm,
                               float *pole_pairs);

/* 获取最近结果；apply=true 时应用到运行参数，save=true 时同时写入 EasyFlash。 */
void DETECT_GetResult(motor_detect_result_t *result);
bool DETECT_ApplyResult(bool save);
const char *DETECT_GetErrorName(motor_detect_error_t error);
void DETECT_Abort(void);
void DETECT_ClearAbortRequest(void);

/* 参数初始化末尾调用：若 Flash 中有合法检测结果，则覆盖代码示例值。 */
void Motor_Detect_LoadStoredParameters(void);
/*
 * 查询偏移检测是否正在运行。
 * 返回 true 时，检测代码正在独占 TIM1/PWM；FOC 中断必须立即退出，
 * 但 ADC-DMA 硬件仍会继续把最新采样值写入 ADC_Value[]。
 */
bool Motor_Detect_IsActive(void);

/* 以下接口由 Motor_DmaInt() 调用，Shell 和普通任务不要直接调用。 */
bool Motor_Detect_ControlIsActive(void);
void Motor_Detect_DmaStep(MOTOR_PARAM_S *motor, float dt);
void Motor_Detect_AbortFromIsr(mc_fault_code fault);
#ifdef __cplusplus
}
#endif

#endif
