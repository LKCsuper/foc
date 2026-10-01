/*
 * @Description: 电机控制通用数学工具（角度与 CRC）
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
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_math.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
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

#ifdef __cplusplus
}
#endif
