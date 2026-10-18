/*
 * @Description: 数学工具函数与内联运算宏
 * @Author: lkc
 *
 * Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se
 *
 * This file is part of the VESC firmware.
 *
 * The VESC firmware is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 * The VESC firmware is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 *     along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef UTILS_MATH_H_
#define UTILS_MATH_H_

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**
 * @description: 计算两角度之差并限制在 [-π, π]。
 * @detail: 无按圈数迭代的循环；任一输入非有限时返回 NaN。
 * @param {float} angle1  角度 1 (rad)
 * @param {float} angle2  角度 2 (rad)
 * @return {float} 角度差 (rad)
 * @author: lkc
 */
float utils_angle_difference_rad(float angle1, float angle2);

/**
 * @description: 快速 atan2 近似。
 * @param {float} y  矢量 y 分量
 * @param {float} x  矢量 x 分量
 * @return {float} 角度 (rad)
 * @author: lkc
 */
float utils_fast_atan2(float y, float x);

/**
 * @description: 计算数据的 CRC-32C (Castagnoli) 校验值。
 * @param {uint8_t} *data  待校验数据指针
 * @param {uint32_t} len   数据长度 (byte)
 * @return {uint32_t} CRC-32C 校验值
 * @author: lkc
 */
uint32_t utils_crc32c(uint8_t *data, uint32_t len);

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
 * 内联工具函数
 * ======================================================================== */

/**
 * @description: 逐步逼近目标值（带步长限制）。
 * @param {float} *value  当前值（就地修改）
 * @param {float} goal    目标值
 * @param {float} step    每拍最大步长（非负）
 * @return {*}
 * @author: lkc
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

/**
 * @description: 角度归一化到 [0°, 360°) 范围。
 * @param {float} *angle  角度 (deg)，就地修改
 * @return {*}
 * @author: lkc
 */
static inline void utils_norm_angle(float *angle) {
	*angle = fmodf(*angle, 360.0f);

	if (*angle < 0.0f) {
		*angle += 360.0f;
	}
}

/**
 * @description: 角度归一化到 [-π, π) 范围（弧度）。
 * @param {float} *angle  角度 (rad)，就地修改
 * @return {*}
 * @author: lkc
 */
static inline void utils_norm_angle_rad(float *angle) {
	while (*angle < -M_PI) { *angle += 2.0f * M_PI; }
	while (*angle >=  M_PI) { *angle -= 2.0f * M_PI; }
}

/**
 * @description: 机械角度转换为电角度。
 * @param {float} mechAngle  机械角度（与输入同单位，通常为 deg 或 rad）
 * @param {int} polePair     极对数
 * @return {float} 电角度（与输入同单位）
 * @author: lkc
 */
static inline float utils_mech_to_electrical_angle(float mechAngle, int polePair)
{
	return mechAngle * polePair;
}

/**
 * @description: 将 float 数值钳位到 [min, max] 区间。
 * @param {float} *number  输入输出值
 * @param {float} min      下限
 * @param {float} max      上限
 * @return {*}
 * @author: lkc
 */
static inline void utils_truncate_number(float *number, float min, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

/**
 * @description: 将 int 数值钳位到 [min, max] 区间。
 * @param {int} *number  输入输出值
 * @param {int} min      下限
 * @param {int} max      上限
 * @return {*}
 * @author: lkc
 */
static inline void utils_truncate_number_int(int *number, int min, int max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

/**
 * @description: 将 float 数值对称钳位到 [-max, max] 区间。
 * @param {float} *number  输入输出值
 * @param {float} max      绝对值上限
 * @return {*}
 * @author: lkc
 */
static inline void utils_truncate_number_abs(float *number, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < -max) {
		*number = -max;
	}
}

/**
 * @description: 线性映射，将 x 从 [in_min, in_max] 映射到 [out_min, out_max]。
 * @param {float} x        输入值
 * @param {float} in_min   输入范围下限
 * @param {float} in_max   输入范围上限
 * @param {float} out_min  输出范围下限
 * @param {float} out_max  输出范围上限
 * @return {float} 映射后的值
 * @author: lkc
 */
static inline float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @description: 线性映射（整数版）。
 * @param {int} x        输入值
 * @param {int} in_min   输入范围下限
 * @param {int} in_max   输入范围上限
 * @param {int} out_min  输出范围下限
 * @param {int} out_max  输出范围上限
 * @return {int} 映射后的值
 * @author: lkc
 */
static inline int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @description: 2D 矢量限幅，超限时等比缩放 x 和 y。
 * @param {float} *x   矢量第一分量（就地修改）
 * @param {float} *y   矢量第二分量（就地修改）
 * @param {float} max  最大允许模长
 * @return {bool} true=发生了限幅
 * @author: lkc
 */
static inline bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag;
	float mag_sq = SQ(*x) + SQ(*y);
	max = fabsf(max);
	/* 明确在限幅圆内时无需sqrt；保留舍入边界、极小值与异常值的原路径。 */
	if (max >= 1e-10f && isfinite(mag_sq) &&
	    mag_sq < SQ(max) * 0.99999976f) {
		return false;
	}
	mag = sqrtf(mag_sq);

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

#ifdef __cplusplus
}
#endif
#endif  /* UTILS_MATH_H_ */
