/*
 * @Description: 2804-220KV 电机铭牌、电气参数与调试默认值
 * @Author: lkc
 * @Note: 图纸来自用户提供的 2804 工程图。控制默认值分成起步、电流环、速度环、位置环，
 *        和另外两款电机是同一套宏。
 *        空轴基础已测；带轮和负载要单独再调，不要几组参数一起放大。
 */
#ifndef MOTOR_2804_220KV_H
#define MOTOR_2804_220KV_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* 图纸铭牌。观测器只用相电阻、相电感，不要拿绕线电阻 5.1 Ω 去顶相电阻。 */
#define MOTOR_2804_220KV_MODEL                  "2804-220KV" /* 型号 */
#define MOTOR_2804_220KV_POLE_COUNT             (14U)        /* 磁极数 */
#define MOTOR_2804_220KV_POLE_PAIRS             (7.0f)       /* 极对数 = 磁极数 / 2 */
#define MOTOR_2804_220KV_SLOT_COUNT             (12U)        /* 定子槽数，12 槽 14 极 */
#define MOTOR_2804_220KV_KV_RPM_PER_V           (220.0f)     /* KV，空载 RPM/V */
#define MOTOR_2804_220KV_RATED_VOLTAGE_V        (12.0f)      /* 额定电压 V */
#define MOTOR_2804_220KV_OPERATING_VOLTAGE_MIN  (7.4f)       /* 工作电压下限 V */
#define MOTOR_2804_220KV_OPERATING_VOLTAGE_MAX  (16.0f)      /* 工作电压上限 V */
#define MOTOR_2804_220KV_TRANSIENT_VOLTAGE_MAX  (20.0f)      /* 瞬时最大电压 V，不作测试电压 */
#define MOTOR_2804_220KV_RATED_CURRENT_A        (0.5f)       /* 额定电流 A */
#define MOTOR_2804_220KV_MAX_CURRENT_A          (2.0f)       /* 图纸最大电流 A，不是首次测试电流 */
#define MOTOR_2804_220KV_RATED_SPEED_RPM        (2600.0f)    /* 额定转速 RPM，12 V */
#define MOTOR_2804_220KV_RATED_TORQUE_NM        (0.03f)      /* 额定转矩 N·m，图纸 300 g·cm */
#define MOTOR_2804_220KV_TORQUE_CONSTANT_NM_A   (0.0434f)    /* 力矩常数 N·m/A */
#define MOTOR_2804_220KV_PHASE_RESISTANCE_OHM   (2.55f)      /* 相电阻 Rs，Ω，观测器用这个 */
#define MOTOR_2804_220KV_WINDING_RESISTANCE_OHM (5.1f)       /* 绕线电阻 Ω，约两相，不要当 Rs */
#define MOTOR_2804_220KV_PHASE_INDUCTANCE_H     (0.00086f)   /* 相电感 Ls，H，即 0.86 mH */
#define MOTOR_2804_220KV_WINDING_INDUCTANCE_H   (0.0028f)    /* 绕线电感 H，即 2.8 mH，不要当 Ls */
#define MOTOR_2804_220KV_FLUX_LINKAGE_WB        (0.0035f)    /* 磁链 Wb */
#define MOTOR_2804_220KV_ROTOR_INERTIA_KGM2     (0.0000037f) /* 转子惯量 kg·m²，小惯量易冲过目标 */

/* motor_param 只认下面这组名字。电阻、电感、磁链单位是 Ω、H、Wb。 */
#define MOTOR_FIXED_MODEL              MOTOR_2804_220KV_MODEL
#define MOTOR_FIXED_POLE_COUNT         MOTOR_2804_220KV_POLE_COUNT
#define MOTOR_FIXED_POLE_PAIRS         MOTOR_2804_220KV_POLE_PAIRS
#define MOTOR_FIXED_PHASE_COUNT        (3U)
#define MOTOR_FIXED_RATED_VOLTAGE      MOTOR_2804_220KV_RATED_VOLTAGE_V
#define MOTOR_FIXED_RATED_RPM          MOTOR_2804_220KV_RATED_SPEED_RPM
#define MOTOR_FIXED_RATED_TORQUE       MOTOR_2804_220KV_RATED_TORQUE_NM
#define MOTOR_FIXED_RATED_CURRENT      MOTOR_2804_220KV_RATED_CURRENT_A
/* 额定转矩乘额定角速度的估算值，只做资料展示，不当保护阈值。 */
#define MOTOR_FIXED_OUTPUT_POWER       (8.17f)
#define MOTOR_FIXED_PEAK_CURRENT       MOTOR_2804_220KV_MAX_CURRENT_A
#define MOTOR_FIXED_TORQUE_CONSTANT    MOTOR_2804_220KV_TORQUE_CONSTANT_NM_A
#define MOTOR_FIXED_LINE_RESISTANCE    MOTOR_2804_220KV_PHASE_RESISTANCE_OHM
#define MOTOR_FIXED_LINE_INDUCTANCE_MH (0.86f) /* 相电感，mH */
#define MOTOR_FIXED_LINE_INDUCTANCE    MOTOR_2804_220KV_PHASE_INDUCTANCE_H
#define MOTOR_FIXED_FLUX_LINKAGE       MOTOR_2804_220KV_FLUX_LINKAGE_WB

/*
 * 起步：开环拖转和对齐，还没进速度环。
 * 0.30 A 躲开 BM75 在 0.10～0.20 A 的采样盲区，也低于 0.5 A 额定电流。
 */
#define MOTOR_OPEN_LOOP_IQ_DEFAULT               (0.30f) /* 开环 I/F 拖转电流 A */
#define MOTOR_ALIGN_CURRENT_DEFAULT              (0.30f) /* 静态 D 轴对齐电流 A */
#define MOTOR_OPEN_LOOP_VBOOST_DEFAULT           (1.0f)  /* V/F 起步补偿电压 V */
#define MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT         (9.0f)  /* 开环 Vq 上限 V */
/* 编码器安装和方向未沿用旧电机，起步助推先关掉。0 表示不注入。 */
#define MOTOR_ENCODER_START_IQ_A                 (0.0f) /* 速度环刚接管时的起步电流 A */
#define MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM    (0.0f) /* 低于此转速给满齿槽助推，RPM */
#define MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM (0.0f) /* 高于此转速退出齿槽助推，RPM */

/* 电流环：起步和闭环共用的最内层。阶跃实测前保持这组保守 PI。 */
#define MOTOR_CURRENT_KP_DEFAULT (0.12f) /* 电流环比例，V/A */
#define MOTOR_CURRENT_KI_DEFAULT (18.0f) /* 电流环积分，V/(A·s) */

/*
 * 速度环：只负责把转速跟上命令。
 * 低电流数不过来，默认把 PI 的电流结果乘 V/A 直接出 Vq。空载电压主要靠反电势前馈。
 */
#define MOTOR_SPEED_KP_DEFAULT            (0.008f)      /* 速度比例，A/RPM */
#define MOTOR_SPEED_KI_DEFAULT            (0.002f)      /* 速度积分，A/(RPM·s) */
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT      (0.20f)       /* 速度环电流上限 A */
#define MOTOR_ENCODER_SPEED_VOLTAGE_MODE  (1U)          /* 1：速度环出电压；0：出电流 */
#define MOTOR_ENCODER_SPEED_VOLT_PER_AMP  (2.55f)       /* 电流换成电压，V/A，约等于相电阻 */
#define MOTOR_ENCODER_SPEED_VOLT_LIMIT_V  (9.0f)        /* 速度环电压上限 V */
#define MOTOR_ENCODER_SPEED_FF_BOOST_V    (0.0f)        /* 低速固定前馈 V，先不加 */
#define MOTOR_ENCODER_SPEED_FF_V_PER_RPM  (0.00454545f) /* 空载反电势前馈，约 1/220 V/RPM */
#define MOTOR_ENCODER_PLL_BW_RAD_S        (30.0f)       /* 速度 PLL 带宽 rad/s，压低齿槽纹波 */

/*
 * 位置环：外层把角度误差变成转速，内层速度 PI 再出电压。
 * 空轴用低增益、低电压、低加速度。2026-10-04 用 0.05 / 0.20、2 V 会冲过目标并打满反向电压。
 */
#define MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG (1.0f)   /* 角度误差换成转速，RPM/deg */
#define MOTOR_POSITION_MAX_RPM_DEFAULT        (2.0f)   /* 位置运动速度上限 RPM */
#define MOTOR_POSITION_MAX_RPM_ALLOWED        (2.0f)   /* 允许提到的最高速度 RPM */
#define MOTOR_POSITION_VEL_KP_DEFAULT         (0.020f) /* 位置环内的速度比例，V/RPM */
#define MOTOR_POSITION_VEL_KI_DEFAULT         (0.02f)  /* 位置环内的速度积分，V/(RPM·s) */
#define MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT  (0.35f)  /* 位置环电压上限 V */
#define MOTOR_POSITION_ACCEL_RPM_S_DEFAULT    (30.0f)  /* 速度变化上限 RPM/s */
#define MOTOR_POSITION_FF_V_PER_RPM_DEFAULT   (0.0f)   /* 位置环速度前馈 V/RPM，先关 */
#define MOTOR_POSITION_HOLD_ENTER_DEG         (2.0f)   /* 误差小于此角度进入保持，deg */
#define MOTOR_POSITION_HOLD_EXIT_DEG          (3.0f)   /* 误差大于此角度退出保持，deg */
#define MOTOR_POSITION_HYBRID_SPEED_ENABLE    (0U)     /* 0：禁止低速切电流环，盲区里会失稳 */
#define MOTOR_POSITION_SMALL_TEST_SEQUENCE    (1U)     /* 1：允许 0°→2°→5° 小行程测试 */
/* 大角度先走加减速轨迹，再交给位置环，不在终点用低分辨率电流硬刹。 */
#define MOTOR_JOINT_TRAJECTORY_ENABLE         (1U)     /* 1：关节位置命令走轨迹 */
#define MOTOR_JOINT_TRAJECTORY_VEL_DEG_S      (60.0f)  /* 轨迹最高速度 deg/s，即 10 RPM */
#define MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2   (120.0f) /* 轨迹加速度 deg/s² */
#define MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2   (180.0f) /* 轨迹减速度 deg/s²，提前制动 */

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
