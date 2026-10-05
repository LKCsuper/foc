/*
 * @Description: C2208-100T 电机铭牌与调试默认值
 * @Author: lkc
 * @Note: 图纸来自 Doc/motor/C2208-100T/2208电机工程图.png。
 *        控制默认值分成起步、电流环、速度环、位置环，和另外两款电机是同一套宏。
 */
#ifndef MOTOR_C2208_100T_H
#define MOTOR_C2208_100T_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* 图纸铭牌。电阻只标了 21.2 Ω，没写清单相还是线间，观测器口径未确认。 */
#define MOTOR_C2208_100T_MODEL              "C2208-100T" /* 型号 */
#define MOTOR_C2208_100T_TYPE               "BLDC"       /* 直流无刷电机 */
#define MOTOR_C2208_100T_POLE_COUNT         (14U)        /* 磁极数 */
#define MOTOR_C2208_100T_POLE_PAIRS         (7.0f)       /* 极对数 = 磁极数 / 2 */
#define MOTOR_C2208_100T_SLOT_COUNT         (12U)        /* 定子槽数 */
#define MOTOR_C2208_100T_PHASE_COUNT        (3U)         /* 相数 */
#define MOTOR_C2208_100T_KV_RPM_PER_V       (120.0f)     /* KV，空载 RPM/V */
#define MOTOR_C2208_100T_RATED_VOLTAGE_V    (12.0f)      /* 额定电压 V */
#define MOTOR_C2208_100T_MAX_VOLTAGE_V      (24.0f)      /* 最大电压 V */
#define MOTOR_C2208_100T_RATED_CURRENT_A    (0.8f)       /* 额定电流 A */
#define MOTOR_C2208_100T_MAX_CURRENT_A      (4.5f)       /* 最大电流 A */
#define MOTOR_C2208_100T_RATED_POWER_W      (14.0f)      /* 额定功率 W */
#define MOTOR_C2208_100T_RATED_SPEED_RPM    (1800.0f)    /* 额定转速 RPM */
#define MOTOR_C2208_100T_RATED_TORQUE_NM    (0.03f)      /* 额定转矩约值 N·m */
/* 工程图只写“电阻 21.2 Ω”，没说明是单相还是两相线间。口径确认前不要当成观测器相电阻。 */
#define MOTOR_C2208_100T_RESISTANCE_OHM     (21.2f)      /* 图纸电阻 Ω */
#define MOTOR_C2208_100T_PHASE_INDUCTANCE_MH (5.3f)      /* 图纸相电感 mH */
#define MOTOR_C2208_100T_PHASE_INDUCTANCE_H (0.0053f)    /* 相电感 H */
#define MOTOR_C2208_100T_WINDING_TURNS      (100U)       /* 每组绕组匝数 */
#define MOTOR_C2208_100T_WIRE_DIAMETER_MM   (0.14f)      /* 漆包线线径 mm */
#define MOTOR_C2208_100T_WINDING_CONNECTION "STAR"       /* 星形接法 */
#define MOTOR_C2208_100T_WEIGHT_G           (38.0f)      /* 重量 g */
#define MOTOR_C2208_100T_OUTER_DIAMETER_MM  (27.5f)      /* 外径 mm */
#define MOTOR_C2208_100T_BODY_HEIGHT_MM     (26.5f)      /* 机身高度 mm */
#define MOTOR_C2208_100T_TOTAL_HEIGHT_MM    (28.0f)      /* 总高度 mm */
#define MOTOR_C2208_100T_SHAFT_DIAMETER_MM  (3.0f)       /* 输出轴直径 mm */
/* 动力线没有固定 U/V/W。MT6701 手册里的 U/V/W 是编码器换相，不是这里的三相线。 */

/* motor_param 只认下面这组名字。电阻、电感、磁链单位是 Ω、H、Wb。 */
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
/* 由 120 RPM/V 估算，未做反电势实测，只作低功率初值。 */
#define MOTOR_FIXED_FLUX_LINKAGE       (0.00928f)

/*
 * 起步：开环拖转和对齐，还没进速度环。
 * 开环电流未单独标定，沿用通用默认值。编码器低速助推是本电机实测值。
 */
#define MOTOR_OPEN_LOOP_IQ_DEFAULT               (0.6f)  /* 开环 I/F 拖转电流 A */
#define MOTOR_ALIGN_CURRENT_DEFAULT              (0.5f)  /* 静态 D 轴对齐电流 A */
#define MOTOR_OPEN_LOOP_VBOOST_DEFAULT           (5.0f)  /* V/F 起步补偿电压 V */
#define MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT         (18.0f) /* 开环 Vq 上限 V */
/* 30 RPM、Kp=0.001 时比例项只有 0.03 A，不够克服齿槽。接管时只预置差额。 */
#define MOTOR_ENCODER_START_IQ_A                 (0.08f) /* 速度环刚接管时的起步电流 A */
#define MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM    (10.0f) /* 低于此转速给满齿槽助推，RPM */
#define MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM (20.0f) /* 高于此转速退出齿槽助推，RPM */

/* 电流环：起步和闭环共用的最内层。未单独标定，沿用通用 PI。 */
#define MOTOR_CURRENT_KP_DEFAULT (0.12f) /* 电流环比例，V/A */
#define MOTOR_CURRENT_KI_DEFAULT (18.0f) /* 电流环积分，V/(A·s) */

/*
 * 速度环：只负责把转速跟上命令。
 * 低电流数不过来，直接出 Vq。2026-09 用这组 PI 在 10/30 RPM 连续跑稳。
 */
#define MOTOR_SPEED_KP_DEFAULT            (0.008f)     /* 速度比例，A/RPM */
#define MOTOR_SPEED_KI_DEFAULT            (0.002f)     /* 速度积分，A/(RPM·s) */
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT      (0.20f)      /* 速度环电流上限 A */
#define MOTOR_ENCODER_SPEED_VOLTAGE_MODE  (1U)         /* 1：速度环出电压；0：出电流 */
#define MOTOR_ENCODER_SPEED_VOLT_PER_AMP  (5.0f)       /* 电流换成电压，V/A */
#define MOTOR_ENCODER_SPEED_VOLT_LIMIT_V  (13.8f)      /* 速度环电压上限 V */
/*
 * 空载前馈。1800 RPM 时 PI 顶在 -0.20 A，实际约 1875 RPM、约 11.17 V。
 * 0.00586 V/RPM 让前馈落在这个电压上，PI 再修偏差。30 RPM 基础前馈约 0.343 V。
 */
#define MOTOR_ENCODER_SPEED_FF_BOOST_V    (0.1666667f) /* 低速固定前馈 V */
#define MOTOR_ENCODER_SPEED_FF_V_PER_RPM  (0.00586f)   /* 空载反电势前馈 V/RPM */
#define MOTOR_ENCODER_PLL_BW_RAD_S        (30.0f)      /* 速度 PLL 带宽 rad/s，压低齿槽纹波 */

/*
 * 位置环：外层把角度误差变成转速，内层速度 PI 再出电压。
 * 内层电压 PI 是 2026-09 位置保持参数。角度比例和轨迹未单独收紧。
 */
#define MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG (20.0f * 60.0f / 360.0f) /* 角度误差换成转速，约 3.33 RPM/deg */
#define MOTOR_POSITION_MAX_RPM_DEFAULT        (60.0f)   /* 位置运动速度上限 RPM */
#define MOTOR_POSITION_MAX_RPM_ALLOWED        (1800.0f) /* 允许提到的最高速度 RPM */
#define MOTOR_POSITION_VEL_KP_DEFAULT         (0.050f)  /* 位置环内的速度比例，V/RPM */
#define MOTOR_POSITION_VEL_KI_DEFAULT         (0.20f)   /* 位置环内的速度积分，V/(RPM·s) */
#define MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT  (2.0f)    /* 位置环电压上限 V */
#define MOTOR_POSITION_ACCEL_RPM_S_DEFAULT    (30000.0f) /* 速度变化上限 RPM/s */
#define MOTOR_POSITION_FF_V_PER_RPM_DEFAULT   (0.0f)    /* 位置环速度前馈 V/RPM */
#define MOTOR_POSITION_HOLD_ENTER_DEG         (2.0f)    /* 误差小于此角度进入保持，deg */
#define MOTOR_POSITION_HOLD_EXIT_DEG          (3.0f)    /* 误差大于此角度退出保持，deg */
#define MOTOR_POSITION_HYBRID_SPEED_ENABLE    (0U)      /* 0：禁止低速切电流环 */
#define MOTOR_POSITION_SMALL_TEST_SEQUENCE    (0U)      /* 0：不跑 0°→2°→5° 小行程测试 */
/* 0：大角度仍直接交给位置环，不走加减速轨迹。 */
#define MOTOR_JOINT_TRAJECTORY_ENABLE         (0U)     /* 1：关节位置命令走轨迹 */
#define MOTOR_JOINT_TRAJECTORY_VEL_DEG_S      (180.0f) /* 轨迹最高速度 deg/s */
#define MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2   (360.0f) /* 轨迹加速度 deg/s² */
#define MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2   (540.0f) /* 轨迹减速度 deg/s² */

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
