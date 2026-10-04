/*
 * @Description: 2804-220KV 电机铭牌、电气参数与首次调试默认值
 * @Author: lkc
 * @Note: 参数来源为用户提供的 2804 工程图；控制默认值尚未上板验证。
 */
#ifndef MOTOR_2804_220KV_H
#define MOTOR_2804_220KV_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_2804_220KV_MODEL                 "2804-220KV"
#define MOTOR_2804_220KV_POLE_COUNT            (14U)
#define MOTOR_2804_220KV_POLE_PAIRS            (7.0f)
#define MOTOR_2804_220KV_SLOT_COUNT             (12U)
#define MOTOR_2804_220KV_KV_RPM_PER_V          (220.0f)
#define MOTOR_2804_220KV_RATED_VOLTAGE_V       (12.0f)
#define MOTOR_2804_220KV_OPERATING_VOLTAGE_MIN (7.4f)
#define MOTOR_2804_220KV_OPERATING_VOLTAGE_MAX (16.0f)
#define MOTOR_2804_220KV_TRANSIENT_VOLTAGE_MAX (20.0f)
#define MOTOR_2804_220KV_RATED_CURRENT_A       (0.5f)
#define MOTOR_2804_220KV_MAX_CURRENT_A         (2.0f)
#define MOTOR_2804_220KV_RATED_SPEED_RPM       (2600.0f)
#define MOTOR_2804_220KV_RATED_TORQUE_NM       (0.03f)
#define MOTOR_2804_220KV_TORQUE_CONSTANT_NM_A  (0.0434f)
#define MOTOR_2804_220KV_PHASE_RESISTANCE_OHM  (2.55f)
#define MOTOR_2804_220KV_WINDING_RESISTANCE_OHM (5.1f)
#define MOTOR_2804_220KV_PHASE_INDUCTANCE_H    (0.00086f)
#define MOTOR_2804_220KV_WINDING_INDUCTANCE_H  (0.0028f)
#define MOTOR_2804_220KV_FLUX_LINKAGE_WB       (0.0035f)
#define MOTOR_2804_220KV_ROTOR_INERTIA_KGM2    (0.0000037f)

/* motor_param 使用的统一铭牌接口。观测器使用图纸明确给出的相参数 Rs/Ls。 */
#define MOTOR_FIXED_MODEL              MOTOR_2804_220KV_MODEL
#define MOTOR_FIXED_POLE_COUNT         MOTOR_2804_220KV_POLE_COUNT
#define MOTOR_FIXED_POLE_PAIRS         MOTOR_2804_220KV_POLE_PAIRS
#define MOTOR_FIXED_PHASE_COUNT        (3U)
#define MOTOR_FIXED_RATED_VOLTAGE      MOTOR_2804_220KV_RATED_VOLTAGE_V
#define MOTOR_FIXED_RATED_RPM          MOTOR_2804_220KV_RATED_SPEED_RPM
#define MOTOR_FIXED_RATED_TORQUE       MOTOR_2804_220KV_RATED_TORQUE_NM
#define MOTOR_FIXED_RATED_CURRENT      MOTOR_2804_220KV_RATED_CURRENT_A
/* 由额定转矩×额定角速度估算，仅用于资料展示，不作为保护阈值。 */
#define MOTOR_FIXED_OUTPUT_POWER       (8.17f)
#define MOTOR_FIXED_PEAK_CURRENT       MOTOR_2804_220KV_MAX_CURRENT_A
#define MOTOR_FIXED_TORQUE_CONSTANT    MOTOR_2804_220KV_TORQUE_CONSTANT_NM_A
#define MOTOR_FIXED_LINE_RESISTANCE    MOTOR_2804_220KV_PHASE_RESISTANCE_OHM
#define MOTOR_FIXED_LINE_INDUCTANCE_MH (0.86f)
#define MOTOR_FIXED_LINE_INDUCTANCE    MOTOR_2804_220KV_PHASE_INDUCTANCE_H
#define MOTOR_FIXED_FLUX_LINKAGE       MOTOR_2804_220KV_FLUX_LINKAGE_WB

/* 首次上板的保守初值：避开 BM75 0.10～0.20 A 盲区，同时低于 0.5 A 额定电流。 */
#define MOTOR_OPEN_LOOP_IQ_DEFAULT    (0.30f)
#define MOTOR_ALIGN_CURRENT_DEFAULT   (0.30f)
#define MOTOR_OPEN_LOOP_VBOOST_DEFAULT (1.0f)
#define MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT (6.0f)
/* 暂沿用已验证框架的保守 PI，实测阶跃前不得按理论带宽激进放大。 */
#define MOTOR_CURRENT_KP_DEFAULT      (0.12f)
#define MOTOR_CURRENT_KI_DEFAULT      (18.0f)
#define MOTOR_SPEED_KP_DEFAULT        (0.003f)
#define MOTOR_SPEED_KI_DEFAULT        (0.01f)
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT  (0.30f) /* 速度环 Iq 上限 A，尚未上板验证 */

/* 编码器机械安装、方向和速度前馈未知，首次适配保持关闭。 */
#define MOTOR_ENCODER_START_IQ_A                    (0.0f)
#define MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM       (0.0f)
#define MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM    (0.0f)
#define MOTOR_ENCODER_SPEED_VOLTAGE_MODE             (0U)
#define MOTOR_ENCODER_SPEED_VOLT_PER_AMP             (0.0f)
#define MOTOR_ENCODER_SPEED_VOLT_LIMIT_V             (0.0f)
#define MOTOR_ENCODER_SPEED_FF_BOOST_V               (0.0f)
#define MOTOR_ENCODER_SPEED_FF_V_PER_RPM             (0.0f)
#define MOTOR_ENCODER_PLL_BW_RAD_S                    (100.0f) /* 速度 PLL 带宽 rad/s */

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
