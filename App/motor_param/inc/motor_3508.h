/*
 * @Description: TYI 3508 电机铭牌与无感检测初值
 * @Author: lkc
 * @Note: 图纸在 Doc/motor/3508，同一页含 KV400 与 KV590。极数、槽数、连续电流两档相同。
 *        R/L/磁链是无感检测的大概值，还没和图纸 KV 核对。
 *        控制默认值未上板标定。图纸 25 A 是连续电流，起步电流用下面的开环默认值。
 */
#ifndef MOTOR_3508_H
#define MOTOR_3508_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_sensor_type.h"
/* Private define ------------------------------------------------------------*/

/* 图纸共有铭牌。KV400 与 KV590 的测试电压不同，KV 仍未确认。 */
#define MOTOR_3508_MODEL                "TYI-3508" /* 型号 */
#define MOTOR_SENSOR_TYPE               MOTOR_SENSOR_SENSORLESS /* 本台未接编码器、霍尔 */
#define MOTOR_3508_POLE_COUNT           (14U)      /* 磁极数 */
#define MOTOR_3508_POLE_PAIRS           (7.0f)     /* 极对数 = 磁极数 / 2 */
#define MOTOR_3508_SLOT_COUNT           (12U)      /* 定子槽数，12 槽 14 极 */
#define MOTOR_3508_KV400_RPM_PER_V      (400.0f)   /* 图纸 KV400，空载 RPM/V */
#define MOTOR_3508_KV590_RPM_PER_V      (590.0f)   /* 图纸 KV590，空载 RPM/V */
#define MOTOR_3508_KV400_TEST_VOLTAGE_V (22.2f)    /* KV400 测试电压 V，6S */
#define MOTOR_3508_KV590_TEST_VOLTAGE_V (14.8f)    /* KV590 测试电压 V，4S */
#define MOTOR_3508_MAX_CURRENT_A        (25.0f)    /* 最大连续电流 A */
#define MOTOR_3508_MAX_POWER_W          (370.0f)   /* 最大连续功率 W */
#define MOTOR_3508_WEIGHT_G             (90.0f)    /* 重量 g */
#define MOTOR_3508_STATOR_DIAMETER_MM   (35.0f)    /* 定子外径 mm */
#define MOTOR_3508_STATOR_THICKNESS_MM  (8.0f)     /* 定子厚度 mm */
#define MOTOR_3508_ROTOR_DIAMETER_MM    (42.5f)    /* 转子直径 mm */

/*
 * 无感检测初值：R=0.4740 ohm，L=0.1120 mH，Flux=2.3900 mWb。
 * 观测器单位是 Ω、H、Wb。按 λ=60/(√3·2π·KV·极对数)，2.390 mWb、7 极对约合 330 RPM/V，
 * 和图纸 400、590 都对不上。
 */
#define MOTOR_3508_PHASE_RESISTANCE_OHM (0.4740f) /* 相电阻 Ω */
#define MOTOR_3508_PHASE_INDUCTANCE_MH  (0.1120f) /* 相电感 mH */
#define MOTOR_3508_PHASE_INDUCTANCE_H   (MOTOR_3508_PHASE_INDUCTANCE_MH * 0.001f)
#define MOTOR_3508_FLUX_LINKAGE_WB      (0.002390f) /* 磁链 Wb，即 2.3900 mWb */

/*
 * KV 未确认，开环转速上限先取偏低的 KV400 与 4S 测试电压相乘。
 * 该值低于 KV400@22.2 V 和 KV590@14.8 V 两种配置的空载转速，作保守上限。
 */
#define MOTOR_3508_CONSERVATIVE_RPM_CAP (MOTOR_3508_KV400_RPM_PER_V * MOTOR_3508_KV590_TEST_VOLTAGE_V)

/*
 * motor_param 只认下面这组名字。
 * 确认 KV 后只改上面的 KV 与测试电压两个宏。
 */
#define MOTOR_FIXED_MODEL              MOTOR_3508_MODEL
#define MOTOR_FIXED_POLE_COUNT         MOTOR_3508_POLE_COUNT
#define MOTOR_FIXED_POLE_PAIRS         MOTOR_3508_POLE_PAIRS
#define MOTOR_FIXED_PHASE_COUNT        (3U)
#define MOTOR_FIXED_RATED_VOLTAGE      MOTOR_3508_KV590_TEST_VOLTAGE_V
#define MOTOR_FIXED_RATED_RPM          MOTOR_3508_CONSERVATIVE_RPM_CAP
#define MOTOR_FIXED_RATED_CURRENT      MOTOR_3508_MAX_CURRENT_A
#define MOTOR_FIXED_OUTPUT_POWER       MOTOR_3508_MAX_POWER_W
#define MOTOR_FIXED_PEAK_CURRENT       MOTOR_3508_MAX_CURRENT_A
/* 力矩常数 N·m/A，由 1.5 × 极对数 × 磁链估算。 */
#define MOTOR_FIXED_TORQUE_CONSTANT    (1.5f * MOTOR_3508_POLE_PAIRS * MOTOR_3508_FLUX_LINKAGE_WB)
#define MOTOR_FIXED_LINE_RESISTANCE    MOTOR_3508_PHASE_RESISTANCE_OHM
#define MOTOR_FIXED_LINE_INDUCTANCE_MH MOTOR_3508_PHASE_INDUCTANCE_MH
#define MOTOR_FIXED_LINE_INDUCTANCE    MOTOR_3508_PHASE_INDUCTANCE_H
#define MOTOR_FIXED_FLUX_LINKAGE       MOTOR_3508_FLUX_LINKAGE_WB

/*
 * 起步：开环拖转和对齐，还没进速度环。
 * 1.0 A 高于 BM75 低电流采样盲区，也远低于图纸连续电流 25 A。
 * 相电阻 0.474 Ω，Vq 上限 4 V 时堵转电流大约 8 A，留在 10 A 过流保护以内。
 */
#define MOTOR_OPEN_LOOP_IQ_DEFAULT               (1.0f) /* 开环 I/F 拖转电流 A */
#define MOTOR_ALIGN_CURRENT_DEFAULT              (1.0f) /* 静态 D 轴对齐电流 A */
#define MOTOR_OPEN_LOOP_VBOOST_DEFAULT           (1.0f) /* V/F 起步补偿电压 V */
#define MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT         (4.0f) /* 开环 Vq 上限 V */
#define MOTOR_ENCODER_START_IQ_A                 (0.0f)  /* 速度环刚接管时的起步电流 A，0 表示不注入 */
#define MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM    (0.0f)  /* 低于此转速给满齿槽助推，RPM */
#define MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM (0.0f)  /* 高于此转速退出齿槽助推，RPM */

/* 电流环：起步和闭环共用的最内层。未单独标定，沿用通用 PI。 */
#define MOTOR_CURRENT_KP_DEFAULT (0.12f) /* 电流环比例，V/A */
#define MOTOR_CURRENT_KI_DEFAULT (18.0f) /* 电流环积分，V/(A·s) */

/*
 * 速度环：未上板标定。先走电流速度环，电流上限 2 A，留在连续电流以内。
 */
#define MOTOR_SPEED_KP_DEFAULT           (0.003f) /* 速度比例，A/RPM */
#define MOTOR_SPEED_KI_DEFAULT           (0.01f)  /* 速度积分，A/(RPM·s) */
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT     (2.0f)   /* 速度环电流上限 A */
#define MOTOR_ENCODER_SPEED_VOLTAGE_MODE (0U)     /* 1：速度环出电压；0：出电流 */
#define MOTOR_ENCODER_SPEED_VOLT_PER_AMP (0.4740f) /* 电流换成电压，V/A，约等于相电阻 */
#define MOTOR_ENCODER_SPEED_VOLT_LIMIT_V (0.0f)   /* 速度环电压上限 V */
#define MOTOR_ENCODER_SPEED_FF_BOOST_V   (0.0f)   /* 低速固定前馈 V */
#define MOTOR_ENCODER_SPEED_FF_V_PER_RPM (0.0f)   /* 空载反电势前馈 V/RPM */
#define MOTOR_ENCODER_PLL_BW_RAD_S       (100.0f) /* 速度 PLL 带宽 rad/s */

/*
 * 位置环：外层把角度误差变成转速，内层速度 PI 再出电压。
 * 尚未单独标定，沿用通用位置参数。轨迹默认关闭。
 */
#define MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG (20.0f * 60.0f / 360.0f) /* 角度误差换成转速，约 3.33 RPM/deg */
#define MOTOR_POSITION_MAX_RPM_DEFAULT        (60.0f)    /* 位置运动速度上限 RPM */
#define MOTOR_POSITION_MAX_RPM_ALLOWED        (1800.0f)  /* 允许提到的最高速度 RPM */
#define MOTOR_POSITION_VEL_KP_DEFAULT         (0.050f)   /* 位置环内的速度比例，V/RPM */
#define MOTOR_POSITION_VEL_KI_DEFAULT         (0.20f)    /* 位置环内的速度积分，V/(RPM·s) */
#define MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT  (2.0f)     /* 位置环电压上限 V */
#define MOTOR_POSITION_ACCEL_RPM_S_DEFAULT    (30000.0f) /* 速度变化上限 RPM/s */
#define MOTOR_POSITION_FF_V_PER_RPM_DEFAULT   (0.0f)     /* 位置环速度前馈 V/RPM */
#define MOTOR_POSITION_HOLD_ENTER_DEG         (2.0f)     /* 误差小于此角度进入保持，deg */
#define MOTOR_POSITION_HOLD_EXIT_DEG          (3.0f)     /* 误差大于此角度退出保持，deg */
#define MOTOR_POSITION_HYBRID_SPEED_ENABLE    (0U)       /* 0：禁止低速切电流环 */
#define MOTOR_POSITION_SMALL_TEST_SEQUENCE    (0U)       /* 0：不跑 0°→2°→5° 小行程测试 */
#define MOTOR_JOINT_TRAJECTORY_ENABLE         (0U)       /* 0：大角度不走轨迹 */
#define MOTOR_JOINT_TRAJECTORY_VEL_DEG_S      (180.0f)   /* 轨迹最高速度 deg/s */
#define MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2   (360.0f)   /* 轨迹加速度 deg/s² */
#define MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2   (540.0f)   /* 轨迹减速度 deg/s² */

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
