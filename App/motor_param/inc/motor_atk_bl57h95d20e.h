/*
 * @Description: ATK-BL57H95D20E 电机铭牌与调试默认值
 * @Author: lkc
 * @Note: 铭牌直接写在 MOTOR_FIXED_*。控制默认值分成起步、电流环、速度环、位置环，
 *        和另外两款电机是同一套宏。低速助推尚未上板标定，保持关闭。
 */
#ifndef MOTOR_ATK_BL57H95D20E_H
#define MOTOR_ATK_BL57H95D20E_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* 铭牌。本电机没有单独的图纸宏，固件直接读下面这组名字。电阻、电感、磁链单位是 Ω、H、Wb。 */
#define MOTOR_FIXED_MODEL              "ATK-BL57H95D20E" /* 型号 */
#define MOTOR_FIXED_POLE_COUNT         (4U)              /* 磁极数 */
#define MOTOR_FIXED_POLE_PAIRS         (2.0f)            /* 极对数 = 磁极数 / 2 */
#define MOTOR_FIXED_PHASE_COUNT        (3U)              /* 相数 */
#define MOTOR_FIXED_RATED_VOLTAGE      (24.0f)           /* 额定电压 V */
#define MOTOR_FIXED_RATED_RPM          (3000.0f)         /* 额定转速 RPM */
#define MOTOR_FIXED_RATED_TORQUE       (0.33f)           /* 额定转矩 N·m */
#define MOTOR_FIXED_RATED_CURRENT      (6.6f)            /* 额定电流 A */
#define MOTOR_FIXED_OUTPUT_POWER       (105.0f)          /* 输出功率 W */
#define MOTOR_FIXED_PEAK_TORQUE        (4.2f)            /* 峰值转矩 N·m */
#define MOTOR_FIXED_PEAK_CURRENT       (19.8f)           /* 峰值电流 A */
#define MOTOR_FIXED_TORQUE_CONSTANT    (0.063f)          /* 力矩常数 N·m/A */
#define MOTOR_FIXED_BACK_EMF           (6.6f)            /* 线间 RMS 反电势 V/kRPM */
#define MOTOR_FIXED_LINE_RESISTANCE    (0.4f)            /* 线电阻 Ω */
#define MOTOR_FIXED_LINE_INDUCTANCE_MH (0.8f)            /* 线电感 mH */
#define MOTOR_FIXED_LINE_INDUCTANCE    (0.0008f)         /* 线电感 H，即 0.8 mH */
#define MOTOR_FIXED_INERTIA            (210.0f)          /* 转动惯量 g·cm² */
#define MOTOR_FIXED_BODY_LENGTH        (95.0f)           /* 机身长度 mm */
#define MOTOR_FIXED_WEIGHT             (1.0f)            /* 重量 kg */
/* 磁链由线间 RMS 反电势换算：λ = Ke / (128.25498 × 极对数)。6.6 V/kRPM、2 极对 → 25.73 mWb。 */
#define MOTOR_FIXED_FLUX_LINKAGE       (0.02573f)

/*
 * 起步：开环拖转和对齐，还没进速度环。
 * 开环电流未单独标定，沿用通用默认值。编码器起步助推关闭。
 */
#define MOTOR_OPEN_LOOP_IQ_DEFAULT               (0.6f)  /* 开环 I/F 拖转电流 A */
#define MOTOR_ALIGN_CURRENT_DEFAULT              (0.5f)  /* 静态 D 轴对齐电流 A */
#define MOTOR_OPEN_LOOP_VBOOST_DEFAULT           (5.0f)  /* V/F 起步补偿电压 V */
#define MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT         (18.0f) /* 开环 Vq 上限 V */
#define MOTOR_ENCODER_START_IQ_A                 (0.0f)  /* 速度环刚接管时的起步电流 A，0 表示不注入 */
#define MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM    (0.0f)  /* 低于此转速给满齿槽助推，RPM */
#define MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM (0.0f)  /* 高于此转速退出齿槽助推，RPM */

/* 电流环：起步和闭环共用的最内层。未单独标定，沿用通用 PI。 */
#define MOTOR_CURRENT_KP_DEFAULT (0.12f) /* 电流环比例，V/A */
#define MOTOR_CURRENT_KI_DEFAULT (18.0f) /* 电流环积分，V/(A·s) */

/*
 * 速度环：只负责把转速跟上命令。
 * 本电机走电流速度环，电压换算保持关闭。PI 沿用原来的电流环参数。
 */
#define MOTOR_SPEED_KP_DEFAULT            (0.003f) /* 速度比例，A/RPM */
#define MOTOR_SPEED_KI_DEFAULT            (0.01f)  /* 速度积分，A/(RPM·s) */
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT      (2.0f)   /* 速度环电流上限 A */
#define MOTOR_ENCODER_SPEED_VOLTAGE_MODE  (0U)     /* 1：速度环出电压；0：出电流 */
#define MOTOR_ENCODER_SPEED_VOLT_PER_AMP  (0.0f)   /* 电流换成电压，V/A，电压模式才用 */
#define MOTOR_ENCODER_SPEED_VOLT_LIMIT_V  (0.0f)   /* 速度环电压上限 V */
#define MOTOR_ENCODER_SPEED_FF_BOOST_V    (0.0f)   /* 低速固定前馈 V */
#define MOTOR_ENCODER_SPEED_FF_V_PER_RPM  (0.0f)   /* 空载反电势前馈 V/RPM */
#define MOTOR_ENCODER_PLL_BW_RAD_S        (100.0f) /* 速度 PLL 带宽 rad/s */

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
