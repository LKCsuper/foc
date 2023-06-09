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
 * @description: angle映射到min 到 max
 * @detail: 
 * Map angle from 0 to 1 in the range min to max. If angle is
 * outside of the range it will be less truncated to the closest
 * angle. Angle units: Degrees
 * @param {float} angle 
 * @param {float} min
 * @param {float} max
 * @return {*}
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
 * @description: 
 * @detail: 
 * 将小于tres的值截断为0
 * Truncate absolute values less than tres to zero. 
 * The value tres will be mapped to 0 and the value max to max.
 * @param {float} *value
 * @param {float} tres
 * @param {float} max
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
 * @description: 
 * @detail: 角度差值-180 到 180
 * Get the difference between two angles. Will always be between -180 and +180 degrees
 * @param {float} angle1 The first angle
 * @param {float} angle2 The second angle
 * @return {float} The difference between the angles
 * @author: lkc
 */
float utils_angle_difference(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -180.0f) difference += 2.0f * 180.0f;
	while (difference > 180.0f) difference -= 2.0f * 180.0f;
	return difference;
}

/**
 * @description: 
 * @detail: 转换为Π的角度差值
 * Get the difference between two angles. Will always be between -pi and +pi radians.
 * @param {float} angle1 The first angle in radians
 * @param {float} angle2 The second angle in radians
 * @return {*} The difference between the angles in radians
 * @author: lkc
 */
float utils_angle_difference_rad(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -M_PI) difference += 2.0f * M_PI;
	while (difference > M_PI) difference -= 2.0f * M_PI;
	return difference;
}

/**
 * @description: 角度平均值,但是这个utils_fast_sincos_better
 * @detail: Takes the average of a number of angles.
 * @param {float} *angles The angles in radians.
 * @param {float} *weights The weight of the summarized angles
 * @param {int} angles_num The number of angles.
 * @return {*} The average angle. 
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
 * @description: Fast atan2
 * @detail: See http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 * @param {float} y
 * @param {float} x
 * @return {*} The angle in radians
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
 * @description: Fast sine and cosine implementation.
 * @detail: See http://lab.polygonal.de/?p=205
 * @param {float} angle The angle in radians
 * WARNING: Don't use too large angles.
 * @param {float} *sin A pointer to store the sine value.
 * @param {float} *cos A pointer to store the cosine value.
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
 * @description: Fast sine and cosine implementation.
 * @detail: See http://lab.polygonal.de/?p=205
 * @param {float} angle The angle in radians
 * WARNING: Don't use too large angles.
 * @param {float} *sin A pointer to store the sine value.
 * @param {float} *cos A pointer to store the cosine value.
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
 * @description: 将int转换为字节 字符串 
 * @detail: Create string representation of the binary content of a byte
 * @param {int} x The byte.
 * @param {char} *b Array to store the string representation in.
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
 * @description: 该函数应该与电源有关系
 * @detail: 
 * @param {float} val
 * @param {float} curve_acc
 * @param {float} curve_brake
 * @param {int} mode
 * @return {*}
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

// Yes, this is only the average...
void utils_fft32_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0;i < 32;i++) {
		*real += real_in[i];
	}

	*real /= 32.0f;
}

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

void utils_fft16_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0;i < 16;i++) {
		*real += real_in[i];
	}

	*real /= 16.0f;
}

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

void utils_fft8_bin0(float *real_in, float *real, float *imag) {
	*real = 0.0f;
	*imag = 0.0f;

	for (int i = 0;i < 8;i++) {
		*real += real_in[i];
	}

	*real /= 8.0f;
}

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

// A mapping of a samsung 30q cell for % remaining capacity vs. voltage from
// 4.2 to 3.2, note that the you lose 15% of the 3Ah rated capacity in this range
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

static int uint16_cmp_func (const void *a, const void *b) {
	return (*(uint16_t*)a - *(uint16_t*)b);
}

uint16_t utils_median_filter_uint16_run(uint16_t *buffer,
		unsigned int *buffer_index, unsigned int filter_len, uint16_t sample) {
	buffer[(*buffer_index)++] = sample;
	*buffer_index %= filter_len;
	uint16_t buffer_sorted[filter_len]; // Assume we have enough stack space
	memcpy(buffer_sorted, buffer, sizeof(uint16_t) * filter_len);
	qsort(buffer_sorted, filter_len, sizeof(uint16_t), uint16_cmp_func);
	return buffer_sorted[filter_len / 2];
}

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
