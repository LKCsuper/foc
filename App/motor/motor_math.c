/*
 * @Description: 电机控制通用数学工具（角度、FFT、CRC、滤波等）
 * @Author: lkc
 */
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
#include "includes.h"
/**
 * @description: 将角度映射到 [min, max] 区间内的归一化比例。
 * @detail: 输入角度超出范围时截断到最近边界；角度单位为度 (deg)。
 * @param {float} angle  待映射角度 (deg)
 * @param {float} min    区间下限 (deg)
 * @param {float} max    区间上限 (deg)
 * @return {float} 映射结果 [0, 1]；min==max 时返回 -1
 * @author: lkc
 */
float utils_map_angle(float angle, float min, float max) {
	if (max == min) {
		return -1;
	}

	float range_pos = max - min;
	utils_norm_angle(&range_pos);
	float range_neg = min - max;
	utils_norm_angle(&range_neg);
	float margin = range_neg / 2.0f;

	angle -= min;
	utils_norm_angle(&angle);
	if (angle > (360.0f - margin)) {
		angle -= 360.0f;
	}

	float res = angle / range_pos;
	utils_truncate_number(&res, 0.0f, 1.0f);

	return res;
}

/**
 * @description: 对输入值施加死区并重新映射到 [-max, max]。
 * @detail: 绝对值小于 tres 的输出置 0；tres 映射到 0，max 映射到 max。
 * @param {float} *value  输入输出值
 * @param {float} tres    死区阈值
 * @param {float} max     满量程
 * @return {*}
 * @author: lkc
 */
void utils_deadband(float *value, float tres, float max) {
	if (fabsf(*value) < tres) {
		*value = 0.0;
	} else {
		float k = max / (max - tres);
		if (*value > 0.0f) {
			*value = k * *value + max * (1.0f - k);
		} else {
			*value = -(k * -*value + max * (1.0f - k));
		}
	}
}

/**
 * @description: 计算两角度之差并归一化到 [-180°, 180°]。
 * @param {float} angle1  角度 1 (deg)
 * @param {float} angle2  角度 2 (deg)
 * @return {float} 最短角度差 (deg)
 * @author: lkc
 */
float utils_angle_difference(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -180.0f) difference += 2.0f * 180.0f;
	while (difference > 180.0f) difference -= 2.0f * 180.0f;
	return difference;
}

/**
 * @description: 计算两弧度角之差并归一化到 [-π, π]。
 * @param {float} angle1  角度 1 (rad)
 * @param {float} angle2  角度 2 (rad)
 * @return {float} 最短角度差 (rad)
 * @author: lkc
 */
float utils_angle_difference_rad(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -M_PI) difference += 2.0f * M_PI;
	while (difference > M_PI) difference -= 2.0f * M_PI;
	return difference;
}

/**
 * @description: 对多个弧度角按权重求圆均值。
 * @detail: 通过 sin/cos 加权求和后 atan2 得到平均角，使用 utils_fast_sincos_better。
 * @param {float} *angles     角度数组 (rad)
 * @param {float} *weights    权重数组
 * @param {int} angles_num    角度个数
 * @return {float} 加权平均角度 (rad)
 * @author: lkc
 */
float utils_avg_angles_rad_fast(IN float *angles, IN float *weights,IN int angles_num) {
	float s_sum = 0.0f;
	float c_sum = 0.0f;

	for (int i = 0; i < angles_num; i++) {
		float s, c;
		utils_fast_sincos_better(angles[i], &s, &c);
		s_sum += s * weights[i];
		c_sum += c * weights[i];
	}

	return utils_fast_atan2(s_sum, c_sum);
}

/**
 * @description: 取三者中间值 float
 * @detail: Get the middle value of three values
 * @param {float} a
 * @param {float} b
 * @param {float} c
 * @return {*} The middle value
 * @author: lkc
 */
float utils_middle_of_3(float a, float b, float c) {
	float middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * @description: 取三者中间值 int
 * @detail: Get the middle value of three values
 * @param {int} a
 * @param {int} b
 * @param {int} c
 * @return {*} The middle value
 * @author: lkc
 */
int utils_middle_of_3_int(int a, int b, int c) {
	int middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * @description: 快速近似 atan2，返回 [-π, π] 弧度角。
 * @param {float} y  y 分量
 * @param {float} x  x 分量
 * @return {float} 极角 (rad)
 * @author: lkc
 */
float utils_fast_atan2(float y, float x) {
	float abs_y = fabsf(y) + 1e-20f; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (M_PI / 4.0f);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * M_PI / 4.0f);
	}

	UTILS_NAN_ZERO(angle);

	if (y < 0) {
		return(-angle);
	} else {
		return(angle);
	}
}

/**
 * @description: 快速近似 sin/cos 计算。
 * @detail: 输入角须先归一化到 [-π, π]；大角度精度会下降。
 * @param {float} angle  输入角度 (rad)
 * @param {float} *sin   sin 输出
 * @param {float} *cos   cos 输出
 * @return {*}
 * @author: lkc
 */
void utils_fast_sincos(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0f * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	// compute sine
	if (angle < 0.0f) {
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;
	} else {
		*sin = 1.27323954f * angle - 0.405284735f * angle * angle;
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5f * M_PI;

	if (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	if (angle < 0.0f) {
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;
	} else {
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;
	}
}

/**
 * @description: 高精度快速近似 sin/cos 计算。
 * @detail: 在 utils_fast_sincos 基础上增加二次修正；输入角须归一化到 [-π, π]。
 * @param {float} angle  输入角度 (rad)
 * @param {float} *sin   sin 输出
 * @param {float} *cos   cos 输出
 * @return {*}
 * @author: lkc
 */
void utils_fast_sincos_better(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0f * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	//compute sine
	if (angle < 0.0f) {
		*sin = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*sin < 0.0f) {
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	} else {
		*sin = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*sin < 0.0f) {
			*sin = 0.225f * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225f * (*sin * *sin - *sin) + *sin;
		}
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5f * M_PI;
	if (angle >  M_PI) {
		angle -= 2.0f * M_PI;
	}

	if (angle < 0.0f) {
		*cos = 1.27323954f * angle + 0.405284735f * angle * angle;

		if (*cos < 0.0f) {
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	} else {
		*cos = 1.27323954f * angle - 0.405284735f * angle * angle;

		if (*cos < 0.0f) {
			*cos = 0.225f * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225f * (*cos * *cos - *cos) + *cos;
		}
	}
}

/**
 * @description: 绝对值最小值
 * @detail: Calculate the values with the lowest magnitude.
 * @param {float} va The first value.
 * @param {float} vb The second value.
 * @return {*} The value with the lowest magnitude.
 * @author: lkc
 */
float utils_min_abs(float va, float vb) {
	float res;
	if (fabsf(va) < fabsf(vb)) {
		res = va;
	} else {
		res = vb;
	}

	return res;
}

/**
 * @description: 绝对值最大值
 * @detail: Calculate the values with the highest magnitude.
 * @param {float} va
 * @param {float} vb
 * @return {*} The value with the highest magnitude.
 * @author: lkc
 */
float utils_max_abs(float va, float vb) {
	float res;
	if (fabsf(va) > fabsf(vb)) {
		res = va;
	} else {
		res = vb;
	}

	return res;
}

/**
 * @description: 将字节整数转换为 8 位二进制字符串。
 * @param {int} x   输入字节 (0~255)
 * @param {char} *b 输出缓冲，至少 9 字节
 * @return {*}
 * @author: lkc
 */
void utils_byte_to_binary(int x, char *b) {
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1) {
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
}

/**
 * @description: 对油门输入施加加速/制动曲线映射。
 * @detail: mode 0=指数、1=自然指数、2=多项式、其他=线性；输出范围 [-1, 1]。
 * @param {float} val         输入值 [-1, 1]
 * @param {float} curve_acc   加速曲线系数
 * @param {float} curve_brake 制动曲线系数
 * @param {int} mode          曲线类型
 * @return {float} 映射后的油门值
 * @author: lkc
 */
float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode) {
	float ret = 0.0f;
	
	if (val < -1.0f) {
		val = -1.0f;
	}

	if (val > 1.0f) {
		val = 1.0f;
	}
	
	float val_a = fabsf(val);

	float curve;
	if (val >= 0.0f) {
		curve = curve_acc;
	} else {
		curve = curve_brake;
	}

	// See
	// http://math.stackexchange.com/questions/297768/how-would-i-create-a-exponential-ramp-function-from-0-0-to-1-1-with-a-single-val
	if (mode == 0) { // Exponential
		if (curve >= 0.0f) {
			ret = 1.0f - powf(1.0f - val_a, 1.0f + curve);
		} else {
			ret = powf(val_a, 1.0f - curve);
		}
	} else if (mode == 1) { // Natural
		if (fabsf(curve) < 1e-10f) {
			ret = val_a;
		} else {
			if (curve >= 0.0f) {
				ret = 1.0f - ((expf(curve * (1.0f - val_a)) - 1.0f) / (expf(curve) - 1.0f));
			} else {
				ret = (expf(-curve * val_a) - 1.0f) / (expf(-curve) - 1.0f);
			}
		}
	} else if (mode == 2) { // Polynomial
		if (curve >= 0.0f) {
			ret = 1.0f - ((1.0f - val_a) / (1.0f + curve * val_a));
		} else {
			ret = val_a / (1.0f - curve * (1.0f - val_a));
		}
	} else { // Linear
		ret = val_a;
	}

	if (val < 0.0f) {
		ret = -ret;
	}

	return ret;
}

/**
 * @description: 计算数据的 CRC-32C (Castagnoli) 校验值
 * @detail: 初值 0xFFFFFFFF，按字节逐位以反射多项式 0x82F63B78 计算，返回取反后的校验值
 * @param {uint8_t} *data    待校验数据指针
 * @param {uint32_t} len     数据长度
 * @return {*}
 * @author: lkc
 */
uint32_t utils_crc32c(uint8_t *data, uint32_t len) {
	uint32_t crc = 0xFFFFFFFF;

	for (uint32_t i = 0; i < len;i++) {
		uint32_t byte = data[i];
		crc = crc ^ byte;

		for (int j = 7;j >= 0;j--) {
			uint32_t mask = -(crc & 1);
			crc = (crc >> 1) ^ (0x82F63B78 & mask);
		}
	}

	return ~crc;
}

/**
 * @description: 计算 32 点 DFT 的 0 号频点（直流分量）。
 * @detail: 输出实部为输入均值，虚部置 0。
 * @param {float} *real_in  输入时域序列（32 点）
 * @param {float} *real     输出实部
 * @param {float} *imag     输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft32_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0;i < 32;i++) {
		*real += real_in[i];
	}

	*real /= 32.0f;
}

/**
 * @description: 计算 32 点 DFT 的 1 号频点（基波）分量
 * @detail: 实部累加 x[i]*cos_32_1[i]，虚部累加 -x[i]*sin_32_1[i]，除以 32 归一化
 * @param {float} *real_in   输入时域序列（32 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft32_bin1(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0;i < 32;i++) {
		*real += real_in[i] * utils_tab_cos_32_1[i];
		*imag -= real_in[i] * utils_tab_sin_32_1[i];
	}
	*real /= 32.0f;
	*imag /= 32.0f;
}

/**
 * @description: 计算 32 点 DFT 的 2 号频点（2 次谐波）分量
 * @detail: 查 utils_tab_*_32_2 表，计算方式同 utils_fft32_bin1
 * @param {float} *real_in   输入时域序列（32 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft32_bin2(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0;i < 32;i++) {
		*real += real_in[i] * utils_tab_cos_32_2[i];
		*imag -= real_in[i] * utils_tab_sin_32_2[i];
	}
	*real /= 32.0f;
	*imag /= 32.0f;
}

/**
 * @description: 计算 16 点 DFT 的 0 号频点（直流分量）
 * @detail: 输出实部为输入均值，虚部置 0
 * @param {float} *real_in   输入时域序列（16 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft16_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0;i < 16;i++) {
		*real += real_in[i];
	}

	*real /= 16.0f;
}

/**
 * @description: 计算 16 点 DFT 的 1 号频点（基波）分量
 * @detail: 在 32 点表上按下标 2*i 取样作 16 点基波表，累加后除以 16 归一化
 * @param {float} *real_in   输入时域序列（16 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft16_bin1(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0;i < 16;i++) {
		*real += real_in[i] * utils_tab_cos_32_1[2 * i];
		*imag -= real_in[i] * utils_tab_sin_32_1[2 * i];
	}
	*real /= 16.0f;
	*imag /= 16.0f;
}

/**
 * @description: 计算 16 点 DFT 的 2 号频点（2 次谐波）分量
 * @detail: 查 *_32_2 表按下标 2*i 取样，计算方式同 utils_fft16_bin1
 * @param {float} *real_in   输入时域序列（16 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft16_bin2(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0;i < 16;i++) {
		*real += real_in[i] * utils_tab_cos_32_2[2 * i];
		*imag -= real_in[i] * utils_tab_sin_32_2[2 * i];
	}
	*real /= 16.0f;
	*imag /= 16.0f;
}

/**
 * @description: 计算 8 点 DFT 的 0 号频点（直流分量）
 * @detail: 输出实部为输入均值，虚部置 0
 * @param {float} *real_in   输入时域序列（8 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft8_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0;i < 8;i++) {
		*real += real_in[i];
	}

	*real /= 8.0f;
}

/**
 * @description: 计算 8 点 DFT 的 1 号频点（基波）分量
 * @detail: 在 32 点表上按下标 4*i 取样作 8 点基波表，累加后除以 8 归一化
 * @param {float} *real_in   输入时域序列（8 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft8_bin1(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0;i < 8;i++) {
		*real += real_in[i] * utils_tab_cos_32_1[4 * i];
		*imag -= real_in[i] * utils_tab_sin_32_1[4 * i];
	}
	*real /= 8.0f;
	*imag /= 8.0f;
}

/**
 * @description: 计算 8 点 DFT 的 2 号频点（2 次谐波）分量
 * @detail: 查 *_32_2 表按下标 4*i 取样，计算方式同 utils_fft8_bin1
 * @param {float} *real_in   输入时域序列（8 点）
 * @param {float} *real      输出实部
 * @param {float} *imag      输出虚部
 * @return {*}
 * @author: lkc
 */
void utils_fft8_bin2(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;
	for (int i = 0;i < 8;i++) {
		*real += real_in[i] * utils_tab_cos_32_2[4 * i];
		*imag -= real_in[i] * utils_tab_sin_32_2[4 * i];
	}
	*real /= 8.0f;
	*imag /= 8.0f;
}

/**
 * @description: 根据归一化锂电池电压估算剩余容量比例。
 * @detail: norm_v 限制在 [0, 1]，对应 Samsung 30Q 4.2V~3.2V 区间的多项式拟合。
 * @param {float} norm_v  归一化电压 [0, 1]
 * @return {float} 剩余容量比例
 * @author: lkc
 */
float utils_batt_liion_norm_v_to_capacity(float norm_v) {
	// constants for polynomial fit of lithium ion battery
	const float li_p[] = {
						  -2.979767f, 5.487810f, -3.501286f, 1.675683f, 0.317147f};
	utils_truncate_number(&norm_v,0.0f,1.0f);
	float v2 = norm_v*norm_v;
	float v3 = v2*norm_v;
	float v4 = v3*norm_v;
	float v5 = v4*norm_v;
	float capacity = li_p[0] * v5 + li_p[1] * v4 + li_p[2] * v3 +
			li_p[3] * v2 + li_p[4] * norm_v;
	return capacity;
}

/**
 * @description: qsort 回调，按 uint16 数值升序比较
 * @param {const void} *a    待比较元素 1
 * @param {const void} *b    待比较元素 2
 * @return {*}
 * @author: lkc
 */
static int uint16_cmp_func (const void *a, const void *b) {
	return (*(uint16_t*)a - *(uint16_t*)b);
}

/**
 * @description: 运行一步 uint16 中值滤波
 * @detail: 将 sample 写入环形缓冲并循环推进写索引，对缓冲排序副本取中值返回
 * @param {uint16_t} *buffer          环形缓冲，长度需为 filter_len
 * @param {unsigned int} *buffer_index 环形缓冲写索引，调用后自动推进取模
 * @param {unsigned int} filter_len   滤波窗口长度
 * @param {uint16_t} sample           本次输入采样值
 * @return {*}
 * @author: lkc
 */
uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
		unsigned int *buffer_index, unsigned int filter_len, uint16_t sample) {
	buffer[(*buffer_index)++] = sample;
	*buffer_index %= filter_len;
	uint16_t buffer_sorted[filter_len]; // Assume we have enough stack space
	memcpy(buffer_sorted, buffer, sizeof(uint16_t) * filter_len);
	qsort(buffer_sorted, filter_len, sizeof(uint16_t), uint16_cmp_func);
	return buffer_sorted[filter_len / 2];
}

/**
 * @description: 用旋转角构成的旋转矩阵 M 旋转 3D 矢量
 * @detail: reverse=false 时 output = M*input；reverse=true 时用 M 的转置（逆旋转）计算
 * @param {float} *input      输入 3D 矢量
 * @param {float} *rotation   三个轴向旋转角（[x,y,z]）
 * @param {float} *output     输出 3D 矢量
 * @param {bool} reverse      是否应用逆旋转
 * @return {*}
 * @author: lkc
 */
void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse) {
	float s1, c1, s2, c2, s3, c3;

	if (rotation[2] != 0.0f) {
		s1 = sinf(rotation[2]);
		c1 = cosf(rotation[2]);
	} else {
		s1 = 0.0f;
		c1 = 1.0f;
	}

	if (rotation[1] != 0.0f) {
		s2 = sinf(rotation[1]);
		c2 = cosf(rotation[1]);
	} else {
		s2 = 0.0f;
		c2 = 1.0f;
	}

	if (rotation[0] != 0.0f) {
		s3 = sinf(rotation[0]);
		c3 = cosf(rotation[0]);
	} else {
		s3 = 0.0f;
		c3 = 1.0f;
	}

	float m11 = c1 * c2;	float m12 = c1 * s2 * s3 - c3 * s1;	float m13 = s1 * s3 + c1 * c3 * s2;
	float m21 = c2 * s1;	float m22 = c1 * c3 + s1 * s2 * s3;	float m23 = c3 * s1 * s2 - c1 * s3;
	float m31 = -s2; 		float m32 = c2 * s3;				float m33 = c2 * c3;

	if (reverse) {
		output[0] = input[0] * m11 + input[1] * m21 + input[2] * m31;
		output[1] = input[0] * m12 + input[1] * m22 + input[2] * m32;
		output[2] = input[0] * m13 + input[1] * m23 + input[2] * m33;
	} else {
		output[0] = input[0] * m11 + input[1] * m12 + input[2] * m13;
		output[1] = input[0] * m21 + input[1] * m22 + input[2] * m23;
		output[2] = input[0] * m31 + input[1] * m32 + input[2] * m33;
	}
}

const float utils_tab_sin_32_1[] = {
	0.000000, 0.195090, 0.382683, 0.555570, 0.707107, 0.831470, 0.923880, 0.980785,
	1.000000, 0.980785, 0.923880, 0.831470, 0.707107, 0.555570, 0.382683, 0.195090,
	0.000000, -0.195090, -0.382683, -0.555570, -0.707107, -0.831470, -0.923880, -0.980785,
	-1.000000, -0.980785, -0.923880, -0.831470, -0.707107, -0.555570, -0.382683, -0.195090};

const float utils_tab_sin_32_2[] = {
	0.000000, 0.382683, 0.707107, 0.923880, 1.000000, 0.923880, 0.707107, 0.382683,
	0.000000, -0.382683, -0.707107, -0.923880, -1.000000, -0.923880, -0.707107, -0.382683,
	-0.000000, 0.382683, 0.707107, 0.923880, 1.000000, 0.923880, 0.707107, 0.382683,
	0.000000, -0.382683, -0.707107, -0.923880, -1.000000, -0.923880, -0.707107, -0.382683};

const float utils_tab_cos_32_1[] = {
	1.000000, 0.980785, 0.923880, 0.831470, 0.707107, 0.555570, 0.382683, 0.195090,
	0.000000, -0.195090, -0.382683, -0.555570, -0.707107, -0.831470, -0.923880, -0.980785,
	-1.000000, -0.980785, -0.923880, -0.831470, -0.707107, -0.555570, -0.382683, -0.195090,
	-0.000000, 0.195090, 0.382683, 0.555570, 0.707107, 0.831470, 0.923880, 0.980785};

const float utils_tab_cos_32_2[] = {
	1.000000, 0.923880, 0.707107, 0.382683, 0.000000, -0.382683, -0.707107, -0.923880,
	-1.000000, -0.923880, -0.707107, -0.382683, -0.000000, 0.382683, 0.707107, 0.923880,
	1.000000, 0.923880, 0.707107, 0.382683, 0.000000, -0.382683, -0.707107, -0.923880,
	-1.000000, -0.923880, -0.707107, -0.382683, -0.000000, 0.382683, 0.707107, 0.923880};
