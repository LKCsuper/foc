/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef UTILS_MATH_H_
#define UTILS_MATH_H_

/*
 * 外部函数声明 — 实现在 motor_math.c 中。
 * 这些是较复杂或依赖查表的工具函数，不适合 inline。
 */
float utils_map_angle(float angle, float min, float max);
void utils_deadband(float *value, float tres, float max);
float utils_angle_difference(float angle1, float angle2);
float utils_angle_difference_rad(float angle1, float angle2);
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num);
float utils_middle_of_3(float a, float b, float c);
int utils_middle_of_3_int(int a, int b, int c);
float utils_fast_atan2(float y, float x);
void utils_fast_sincos(float angle, float *sin, float *cos);
void utils_fast_sincos_better(float angle, float *sin, float *cos);
float utils_min_abs(float va, float vb);
float utils_max_abs(float va, float vb);
void utils_byte_to_binary(int x, char *b);
float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode);
uint32_t utils_crc32c(uint8_t *data, uint32_t len);
void utils_fft32_bin0(float *real_in, float *real, float *imag);
void utils_fft32_bin1(float *real_in, float *real, float *imag);
void utils_fft32_bin2(float *real_in, float *real, float *imag);
void utils_fft16_bin0(float *real_in, float *real, float *imag);
void utils_fft16_bin1(float *real_in, float *real, float *imag);
void utils_fft16_bin2(float *real_in, float *real, float *imag);
void utils_fft8_bin0(float *real_in, float *real, float *imag);
void utils_fft8_bin1(float *real_in, float *real, float *imag);
void utils_fft8_bin2(float *real_in, float *real, float *imag);
float utils_batt_liion_norm_v_to_capacity(float norm_v);
uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
		unsigned int *buffer_index, unsigned int filter_len, uint16_t sample);
void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse);

/* ========================================================================
 * 基础运算宏
 * ======================================================================== */

/* 符号函数：负数返回 -1.0，零或正数返回 1.0。 */
#define SIGN(x)				(((x) < 0.0f) ? -1.0f : 1.0f)

/* 平方：x² */
#define SQ(x)				((x) * (x))

/* 2D 矢量模长：sqrt(x² + y²)，使用单精度 sqrtf。 */
#define NORM2_f(x,y)		(sqrtf(SQ(x) + SQ(y)))

/* ========================================================================
 * 浮点异常检测与处理
 * ======================================================================== */

/* 判断 x 是否为正无穷或负无穷。 */
#define UTILS_IS_INF(x)		((x) == (1.0f / 0.0f) || (x) == (-1.0f / 0.0f))

/* 判断 x 是否为 NaN。NaN 的唯一特性是不等于自身。 */
#define UTILS_IS_NAN(x)		((x) != (x))

/* 若 x 为 NaN 则置零，否则保持原值。防止 NaN 传播污染后续运算。 */
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0f : x)

/* ========================================================================
 * 单位转换宏
 * ======================================================================== */

/* 度 → 弧度 */
#define DEG2RAD_f(deg) ((deg) * (float)(M_PI / 180.0f))
/* 弧度 → 度 */
#define RAD2DEG_f(rad) ((rad) * (float)(180.0f / M_PI))
/* RPM（转/分钟） → rad/s（弧度/秒） */
#define RPM2RADPS_f(rpm) ((rpm) * (float)((2.0f * M_PI) / 60.0f))
/* rad/s → RPM */
#define RADPS2RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0f / (2.0f * M_PI)))

/* ========================================================================
 * 最值宏
 * ======================================================================== */

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

/* 双精度字面量：D(1.5) → (double)1.5L */
#define D(x) 				((double)x##L)

/* ========================================================================
 * 一阶低通滤波器 (指数平滑)
 *
 * UTILS_LP_FAST  — 通用一阶低通，filter_constant 越大响应越快。
 *                   0.0 = 完全冻结，1.0 = 不过滤（直通）。
 *                   value 必须每次调用传入同一变量（就地更新）。
 *
 * UTILS_LP_MOVING_AVG_APPROX — N 点滑动平均的指数近似。
 *                   行为类似 IIR 而非 FIR，但比真正滑动平均省内存、
 *                   省 CPU。N 越大越平滑、响应越慢。
 * ======================================================================== */

#define UTILS_LP_FAST(value, sample, filter_constant) \
	(value -= (filter_constant) * ((value) - (sample)))

#define UTILS_LP_MOVING_AVG_APPROX(value, sample, N) \
	UTILS_LP_FAST(value, sample, 2.0f / ((N) + 1.0f))

/* ========================================================================
 * 数学常量
 * ======================================================================== */

#define M_PI                    (3.14159265358f)
#define M_2PI                   (6.28318530716f)

/* SVPWM / Clarke / Park 变换中使用 */
#define ONE_BY_SQRT3			(0.57735026919f)   /* 1/√3 */
#define TWO_BY_SQRT3			(2.0f * 0.57735026919f) /* 2/√3 */
#define SQRT3_BY_2				(0.86602540378f)   /* √3/2 = cos(30°) */
#define COS_30_DEG				(0.86602540378f)   /* cos(30°) */
#define SIN_30_DEG				(0.5f)             /* sin(30°) */
#define COS_MINUS_30_DEG		(0.86602540378f)   /* cos(-30°) */
#define SIN_MINUS_30_DEG		(-0.5f)            /* sin(-30°) */
#define ONE_BY_SQRT2			(0.7071067811865475f) /* 1/√2 */

/* ========================================================================
 * 正弦/余弦查表 — 32 点分段线性插值用。
 * 实现在 motor_math.c 中。
 * ======================================================================== */
extern const float utils_tab_sin_32_1[];
extern const float utils_tab_sin_32_2[];
extern const float utils_tab_cos_32_1[];
extern const float utils_tab_cos_32_2[];

/* ========================================================================
 * 内联工具函数
 * ======================================================================== */

/*
 * 逐步逼近目标值（带步长限制）。
 * 每次调用最多走 |step| 距离，不会越过 goal。
 * 常用于软启动、转速斜坡、滤波器渐进等场景。
 *
 * @param value  当前值（就地修改）
 * @param goal   目标值
 * @param step   每拍最大步长（非负）
 */
static inline void utils_step_towards(float *value, float goal, float step) {
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

/*
 * 角度归一化到 [0°, 360°) 范围。
 * 例：-30° → 330°，380° → 20°。
 *
 * @param angle 角度（度），就地修改
 */
static inline void utils_norm_angle(float *angle) {
	*angle = fmodf(*angle, 360.0f);

	if (*angle < 0.0f) {
		*angle += 360.0f;
	}
}

/*
 * 角度归一化到 [-π, π) 范围（弧度）。
 * 使用 while 循环而非 fmodf，适合角度增量不大的场景。
 * 警告：传入超大值（如 10000*π）会导致循环次数多，实时性下降。
 *
 * @param angle 角度（弧度），就地修改
 */
static inline void utils_norm_angle_rad(float *angle) {
	while (*angle < -M_PI) { *angle += 2.0f * M_PI; }
	while (*angle >=  M_PI) { *angle -= 2.0f * M_PI; }
}

/*
 * 机械角度 → 电角度。
 * 电角度 = 机械角度 × 极对数。
 *
 * @param mechAngle  机械角度（任意单位，通常为度或弧度）
 * @param polePair   极对数（转子永磁体极数 / 2）
 * @return           电角度（与输入同单位）
 */
static inline float utils_mech_to_electrical_angle(float mechAngle, int polePair)
{
	return mechAngle * polePair;
}

/*
 * 数值钳位（float）：限制 number 在 [min, max] 区间内。
 * 常用于限制占空比（0~1）、电压幅值等。
 */
static inline void utils_truncate_number(float *number, float min, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

/*
 * 数值钳位（int）：限制 number 在 [min, max] 区间内。
 */
static inline void utils_truncate_number_int(int *number, int min, int max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

/*
 * 对称钳位：限制 number 在 [-max, max] 区间内。
 * 等价于 utils_truncate_number(number, -max, max)。
 */
static inline void utils_truncate_number_abs(float *number, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < -max) {
		*number = -max;
	}
}

/*
 * 线性映射：将 x 从 [in_min, in_max] 线性映射到 [out_min, out_max]。
 *
 * 例：x=0.5 在 [0,1] 中 → 映射到 [0, 4095] → 2047.5
 *
 * @param x       输入值
 * @param in_min  输入范围下限
 * @param in_max  输入范围上限
 * @param out_min 输出范围下限
 * @param out_max 输出范围上限
 * @return        映射后的值
 */
static inline float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* utils_map 的整数版本。注意整数除法会截断小数。 */
static inline int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * 2D 矢量限幅：若矢量模长超过 max，按比例等比缩放 x 和 y。
 * 只改幅值不改方向，是 SVPWM 过调制保护和 PI anti-windup 的核心工具。
 *
 * 特殊处理：
 *   - max 取绝对值，防止负限幅
 *   - 矢量模长极小 (<1e-10) 时不会误判为超限（避免除零放大噪声）
 *
 * @param x   [in/out] 矢量第一分量
 * @param y   [in/out] 矢量第二分量
 * @param max 最大允许模长
 * @return    true = 发生了限幅（调用方可据此回退 PI 积分项）
 */
static inline bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag = NORM2_f(*x, *y);
	max = fabsf(max);

	if (mag < 1e-10f) {
		mag = 1e-10f;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

#endif  /* UTILS_MATH_H_ */
