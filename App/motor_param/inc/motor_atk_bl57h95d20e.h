/*
 * @Description: ATK-BL57H95D20E 电机铭牌参数
 * @Author: lkc
 */
#ifndef MOTOR_ATK_BL57H95D20E_H
#define MOTOR_ATK_BL57H95D20E_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_FIXED_MODEL              "ATK-BL57H95D20E" /* 型号 */
#define MOTOR_FIXED_POLE_COUNT         (4U)              /* 磁极数 */
#define MOTOR_FIXED_POLE_PAIRS         (2.0f)            /* 极对数 = 磁极数 / 2 */
#define MOTOR_FIXED_PHASE_COUNT        (3U)              /* 相数 */
#define MOTOR_FIXED_RATED_VOLTAGE      (24.0f)           /* 额定电压 VDC */
#define MOTOR_FIXED_RATED_RPM          (3000.0f)         /* 额定转速 RPM */
#define MOTOR_FIXED_RATED_TORQUE       (0.33f)           /* 额定扭矩 N·m */
#define MOTOR_FIXED_RATED_CURRENT      (6.6f)            /* 额定电流 A */
#define MOTOR_FIXED_OUTPUT_POWER       (105.0f)          /* 输出功率 W */
#define MOTOR_FIXED_PEAK_TORQUE        (4.2f)            /* 峰值扭矩 N·m */
#define MOTOR_FIXED_PEAK_CURRENT       (19.8f)           /* 峰值电流 A */
#define MOTOR_FIXED_TORQUE_CONSTANT    (0.063f)          /* 力矩常数 N·m/A */
#define MOTOR_FIXED_BACK_EMF           (6.6f)            /* 线间 RMS 反电势 V/kRPM */
#define MOTOR_FIXED_LINE_RESISTANCE    (0.4f)            /* 线电阻 Ω */
#define MOTOR_FIXED_LINE_INDUCTANCE_MH (0.8f)            /* 线电感 mH */
#define MOTOR_FIXED_LINE_INDUCTANCE    (0.0008f)         /* 线电感 H，0.8 mH */
#define MOTOR_FIXED_INERTIA            (210.0f)          /* 转动惯量 g·cm^2 */
#define MOTOR_FIXED_BODY_LENGTH        (95.0f)           /* 机身长度 mm */
#define MOTOR_FIXED_WEIGHT             (1.0f)            /* 重量 kg */
/*
 * 磁链由线间 RMS Ke 换算：λ = Ke / (128.25498 × 极对数)。
 * 6.6 V/kRPM、2 极对 → 25.73 mWb。
 */
#define MOTOR_FIXED_FLUX_LINKAGE (0.02573f)

/* ATK 电机尚未上板标定低速起动电流，默认关闭补偿。 */
#define MOTOR_ENCODER_START_IQ_A (0.0f)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
