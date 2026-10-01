/*
 * @Description: C2208-100T 电机铭牌与结构参数
 * @Author: lkc
 * @Note: 参数来源为 Doc/motor/C2208-100T/2208电机工程图.png。
 *        通过 motor_nameplate.h 选择本电机后，统一导出 MOTOR_FIXED_* 参数。
 */
#ifndef MOTOR_C2208_100T_H
#define MOTOR_C2208_100T_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_C2208_100T_MODEL                 "C2208-100T" /* 型号 */
#define MOTOR_C2208_100T_TYPE                  "BLDC"       /* 直流无刷电机 */
#define MOTOR_C2208_100T_POLE_COUNT            (14U)        /* 磁极数 */
#define MOTOR_C2208_100T_POLE_PAIRS            (7.0f)       /* 极对数 = 磁极数 / 2 */
#define MOTOR_C2208_100T_SLOT_COUNT            (12U)        /* 定子槽数 */
#define MOTOR_C2208_100T_PHASE_COUNT           (3U)         /* 相数 */
#define MOTOR_C2208_100T_KV_RPM_PER_V          (120.0f)     /* KV，RPM/V */
#define MOTOR_C2208_100T_RATED_VOLTAGE_V       (12.0f)      /* 额定电压 VDC */
#define MOTOR_C2208_100T_MAX_VOLTAGE_V         (24.0f)      /* 最大电压 VDC */
#define MOTOR_C2208_100T_RATED_CURRENT_A       (0.8f)       /* 额定电流 A */
#define MOTOR_C2208_100T_MAX_CURRENT_A         (4.5f)       /* 最大电流 A */
#define MOTOR_C2208_100T_RATED_POWER_W         (14.0f)      /* 额定功率 W */
#define MOTOR_C2208_100T_RATED_SPEED_RPM       (1800.0f)    /* 额定转速 RPM */
#define MOTOR_C2208_100T_RATED_TORQUE_NM       (0.03f)      /* 额定扭矩约值 N·m */

/*
 * 工程图只标注“电阻 21.2Ω”，未说明是单相电阻还是两相线间电阻。
 * 在实测确认口径前，不得直接用作无感观测器的相电阻参数。
 */
#define MOTOR_C2208_100T_RESISTANCE_OHM        (21.2f)
#define MOTOR_C2208_100T_PHASE_INDUCTANCE_MH   (5.3f)       /* 工程图标注相电感 mH */
#define MOTOR_C2208_100T_PHASE_INDUCTANCE_H    (0.0053f)    /* 相电感 H */

#define MOTOR_C2208_100T_WINDING_TURNS         (100U)       /* 每组绕组匝数 */
#define MOTOR_C2208_100T_WIRE_DIAMETER_MM      (0.14f)      /* 漆包线线径 mm */
#define MOTOR_C2208_100T_WINDING_CONNECTION    "STAR"       /* 星形接法 */
#define MOTOR_C2208_100T_WEIGHT_G              (38.0f)      /* 重量 g */
#define MOTOR_C2208_100T_OUTER_DIAMETER_MM     (27.5f)      /* 外径 mm */
#define MOTOR_C2208_100T_BODY_HEIGHT_MM        (26.5f)      /* 机身高度 mm */
#define MOTOR_C2208_100T_TOTAL_HEIGHT_MM       (28.0f)      /* 总高度 mm */
#define MOTOR_C2208_100T_SHAFT_DIAMETER_MM     (3.0f)       /* 输出轴直径 mm */

/*
 * 编码器速度闭环期望的起动总 Iq（比例项 + 积分预置）。
 * 30 RPM 且 Kp=0.001 A/RPM 时比例项只有 0.03 A，实测不足以克服齿槽转矩。
 * 首次接管时会扣除 Kp*速度误差，只把不足的部分预置到积分；后续由 PI 自由增减。
 */
#define MOTOR_ENCODER_START_IQ_A (0.08f)
/* 强齿槽位置的低速助推：10 RPM 内全量，20 RPM 完全退出。 */
#define MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM    (10.0f)
#define MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM (20.0f)
/* BM75 低电流分辨率不足：C2208 编码器速度环直接输出 Vq。 */
#define MOTOR_ENCODER_SPEED_VOLTAGE_MODE  (1U)
#define MOTOR_ENCODER_SPEED_VOLT_PER_AMP  (5.0f)
#define MOTOR_ENCODER_SPEED_VOLT_LIMIT_V  (13.8f)
/*
 * 速度前馈：低速助推仍是 0.167 V。斜率按 2026-10-01 空载实测：
 * 目标 1800 RPM 时 PI 顶在 -0.20 A，实际约 1875 RPM，对应电压约 11.17 V。
 * 0.00586 V/RPM 使 1800 RPM 的前馈落在这个电压上，±1 V 的 PI 余量还能修正偏差。
 * 30 RPM 基础前馈约 0.343 V。
 */
#define MOTOR_ENCODER_SPEED_FF_BOOST_V   (0.1666667f)
#define MOTOR_ENCODER_SPEED_FF_V_PER_RPM (0.00586f)
/* 低速齿槽明显，降低速度 PLL 带宽；电角度仍直接使用 MT6701 原始角度。 */
#define MOTOR_ENCODER_PLL_BW_RAD_S (30.0f)
/* 2026-09 上板验证：10/30 RPM 编码器电压速度环使用该组参数可连续稳定运行。 */
#define MOTOR_SPEED_KP_DEFAULT       (0.008f)
#define MOTOR_SPEED_KI_DEFAULT       (0.002f)
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT (0.20f)

/* motor_param 使用的统一铭牌接口。 */
#define MOTOR_FIXED_MODEL              MOTOR_C2208_100T_MODEL
#define MOTOR_FIXED_POLE_COUNT         MOTOR_C2208_100T_POLE_COUNT
#define MOTOR_FIXED_POLE_PAIRS         MOTOR_C2208_100T_POLE_PAIRS
#define MOTOR_FIXED_PHASE_COUNT        MOTOR_C2208_100T_PHASE_COUNT
#define MOTOR_FIXED_RATED_VOLTAGE      MOTOR_C2208_100T_RATED_VOLTAGE_V
#define MOTOR_FIXED_RATED_RPM          MOTOR_C2208_100T_RATED_SPEED_RPM
#define MOTOR_FIXED_RATED_TORQUE       MOTOR_C2208_100T_RATED_TORQUE_NM
#define MOTOR_FIXED_RATED_CURRENT      MOTOR_C2208_100T_RATED_CURRENT_A
#define MOTOR_FIXED_OUTPUT_POWER       MOTOR_C2208_100T_RATED_POWER_W
#define MOTOR_FIXED_PEAK_CURRENT       MOTOR_C2208_100T_MAX_CURRENT_A
#define MOTOR_FIXED_LINE_RESISTANCE    MOTOR_C2208_100T_RESISTANCE_OHM
#define MOTOR_FIXED_LINE_INDUCTANCE_MH MOTOR_C2208_100T_PHASE_INDUCTANCE_MH
#define MOTOR_FIXED_LINE_INDUCTANCE    MOTOR_C2208_100T_PHASE_INDUCTANCE_H
/*
 * 由 120 RPM/V 估算线间 Ke=1000/120=8.333 V/kRPM，再按当前观测器约定换算。
 * 该值未经过反电势实测，只能作为首次低功率调试的初值。
 */
#define MOTOR_FIXED_FLUX_LINKAGE (0.00928f)

/*
 * 三根动力线在工程图中没有指定固定 U/V/W 线序，可在低功率、限流条件下标定。
 * MT6701 手册中的 U/V/W 是编码器换相信号，不是这里的三相动力线。
 */
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
