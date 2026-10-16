/*
 * @Description: FOC 坐标变换与 SVPWM 调制（Clarke/Park/逆 Park/SVM）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "common_assert.h"
#include <stddef.h>
#include <math.h>
#include "motor_foc.h"
#include "motor_math.h"
#if defined(__CC_ARM)
#pragma O2
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: Clarke 变换，将三相电流投影到 αβ 静止坐标系。
 * @detail: 三电阻采样时使用完整 Clarke；两电阻硬件按 Ia/Ib 假设三相平衡。
 * @param {float} Ia       A 相电流 (A)
 * @param {float} Ib       B 相电流 (A)
 * @param {float} Ic       C 相电流 (A)
 * @param {float} *Ialpha  输出 α 轴电流 (A)
 * @param {float} *Ibeta   输出 β 轴电流 (A)
 * @return {*}
 * @author: lkc
 */
void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
	/*
	 * 三电阻采样用完整 Clarke；两电阻按 Ia/Ib 假设三相平衡。
	 * 板级默认由 MOTOR_FOC_THREE_SHUNTS 配置（本仓库 HD75 为 1）。
	 */
#if MOTOR_FOC_THREE_SHUNTS
	*Ialpha = (2.0f / 3.0f) * Ia - (1.0f / 3.0f) * Ib - (1.0f / 3.0f) * Ic;
	*Ibeta = (Ib - Ic) * ONE_BY_SQRT3;
#else
	/* 两电阻硬件没有可信的 Ic，按 Ia/Ib 假设三相平衡。 */
	*Ialpha = Ia;
	*Ibeta = ONE_BY_SQRT3 * Ia + TWO_BY_SQRT3 * Ib;
#endif

	return;
}

/**
 * @description: clarke 逆变换，将 αβ 轴电压还原为三相相电压
 * @detail: 输出为相对中性点的相电压
 * @param {float} Ialpha  α 轴电压
 * @param {float} Ibeta   β 轴电压
 * @param {float} *Ia     A 相电压输出
 * @param {float} *Ib     B 相电压输出
 * @param {float} *Ic     C 相电压输出
 * @return {*}
 * @author: lkc
 */
void inverse_clarke(float Ialpha, float Ibeta, float *Ia, float *Ib, float *Ic)
{
    /*
     * 历史写法（已停用）：三相各自加 24/2 = 12 的直流偏置，即把围绕 0 的
     * 双极性相电压平移半个 24 V 母线，凑成单极性区间，便于直接写比较值。
     * 现直流偏置已统一到 SVPWM 级完成（foc_svm/svm_hqd 以 ARR/2 为中心），
     * 此处若再加一次会重复抬升、把占空比整体顶偏，故只保留三相零和的差模输出。
     * 下方三行仅作历史参考，C 侧无调用者；对应 MATLAB foc.inverse_clarke（零和版）。
     */
    // *Ia =  Ialpha + 24.0f / 2;
    // *Ib =  Ibeta * SQRT3_BY_2 - Ialpha / 2 + 24.0 / 2;
    // *Ic =  - Ialpha / 2 - Ibeta * SQRT3_BY_2 + 24.0 / 2;

    *Ia =  Ialpha;
    *Ib =  Ibeta * SQRT3_BY_2 - Ialpha / 2;
    *Ic =  - Ialpha / 2 - Ibeta * SQRT3_BY_2;
	
	return;
}

/**
 * @description: Park 变换，将 αβ 电流投影到与转子同步的 dq 坐标系。
 * @param {float} Ialpha  α 轴电流 (A)
 * @param {float} Ibeta   β 轴电流 (A)
 * @param {float} Theta   电角度 (rad)
 * @param {float} *Id     输出 d 轴电流 (A)
 * @param {float} *Iq     输出 q 轴电流 (A)
 * @return {*}
 * @author: lkc
 */
void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
	float s, c;
	sincos_f32(Theta, &s, &c);
	*Id =   Ialpha * c + Ibeta * s;
	*Iq = - Ialpha * s + Ibeta * c;
}

/**
 * @description: 逆 Park 变换，将 dq 电压还原为 αβ 电压矢量。
 * @param {float} mod_d      d 轴电压 (V)
 * @param {float} mod_q      q 轴电压 (V)
 * @param {float} Theta      电角度 (rad)
 * @param {float} *mod_alpha 输出 α 轴电压 (V)
 * @param {float} *mod_beta  输出 β 轴电压 (V)
 * @return {*}
 * @author: lkc
 */
void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta)
{
	float s, c;
	sincos_f32(Theta, &s, &c);
	/* 保持当前功率板已经上板验证过的电压矢量极性（3e5d564）。 */
	*mod_alpha = -(mod_d * c - mod_q * s);
	*mod_beta  = -(mod_d * s + mod_q * c);
	
	return;
}

/**
 * @description: 空间矢量 PWM 调制，将 αβ 电压归一化为三相占空比。
 * @detail: 输出 tA/tB/tC 为 [0, 1] 归一化占空比；任一相越界时返回 -1。
 * @param {float} alpha  α 轴电压（归一化）
 * @param {float} beta   β 轴电压（归一化）
 * @param {float} *tA    A 相占空比输出
 * @param {float} *tB    B 相占空比输出
 * @param {float} *tC    C 相占空比输出
 * @return {int} 占空比合法返回 0，否则返回 -1
 * @author: lkc
 */
int svm(float alpha, float beta, float* tA, float* tB, float* tC)
{
	int Sextant;

	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			//quadrant I
			if (ONE_BY_SQRT3 * beta > alpha)
				Sextant = 2; //sextant v2-v3
			else
				Sextant = 1; //sextant v1-v2

		} else {
			//quadrant II
			if (-ONE_BY_SQRT3 * beta > alpha)
				Sextant = 3; //sextant v3-v4
			else
				Sextant = 2; //sextant v2-v3
		}
	} else {
		if (alpha >= 0.0f) {
			//quadrant IV
			if (-ONE_BY_SQRT3 * beta > alpha)
				Sextant = 5; //sextant v5-v6
			else
				Sextant = 6; //sextant v6-v1
		} else {
			//quadrant III
			if (ONE_BY_SQRT3 * beta > alpha)
				Sextant = 4; //sextant v4-v5
			else
				Sextant = 5; //sextant v5-v6
		}
	}

	switch (Sextant) {
		// sextant v1-v2
		case 1: {
			// Vector on-times
			float t1 = alpha - ONE_BY_SQRT3 * beta;
			float t2 = TWO_BY_SQRT3 * beta;

			// PWM timings
			*tA = (1.0f - t1 - t2) * 0.5f;
			*tB = *tA + t1;
			*tC = *tB + t2;
		} break;

		// sextant v2-v3
		case 2: {
			// Vector on-times
			float t2 = alpha + ONE_BY_SQRT3 * beta;
			float t3 = -alpha + ONE_BY_SQRT3 * beta;

			// PWM timings
			*tB = (1.0f - t2 - t3) * 0.5f;
			*tA = *tB + t3;
			*tC = *tA + t2;
		} break;

		// sextant v3-v4
		case 3: {
			// Vector on-times
			float t3 = TWO_BY_SQRT3 * beta;
			float t4 = -alpha - ONE_BY_SQRT3 * beta;

			// PWM timings
			*tB = (1.0f - t3 - t4) * 0.5f;
			*tC = *tB + t3;
			*tA = *tC + t4;
		} break;

		// sextant v4-v5
		case 4: {
			// Vector on-times
			float t4 = -alpha + ONE_BY_SQRT3 * beta;
			float t5 = -TWO_BY_SQRT3 * beta;

			// PWM timings
			*tC = (1.0f - t4 - t5) * 0.5f;
			*tB = *tC + t5;
			*tA = *tB + t4;
		} break;

		// sextant v5-v6
		case 5: {
			// Vector on-times
			float t5 = -alpha - ONE_BY_SQRT3 * beta;
			float t6 = alpha - ONE_BY_SQRT3 * beta;

			// PWM timings
			*tC = (1.0f - t5 - t6) * 0.5f;
			*tA = *tC + t5;
			*tB = *tA + t6;
		} break;

		// sextant v6-v1
		case 6: {
			// Vector on-times
			float t6 = -TWO_BY_SQRT3 * beta;
			float t1 = alpha + ONE_BY_SQRT3 * beta;

			// PWM timings
			*tA = (1.0f - t6 - t1) * 0.5f;
			*tC = *tA + t1;
			*tB = *tC + t6;
		} break;
	}

	// if any of the results becomes NaN, result_valid will evaluate to false
	int result_valid =
			*tA >= 0.0f && *tA <= 1.0f
		&& *tB >= 0.0f && *tB <= 1.0f
		&& *tC >= 0.0f && *tC <= 1.0f;
	
	return result_valid ? 0 : -1;
}

/**
 * @description: 空间矢量 PWM 调制，输出定时器比较值与扇区号。
 * @detail: 输入 αβ 电压与 PWM 计数满量程，计算三路 CCR 占空比；幅值不应超过 sqrt(3)/2 以免过调制。
 * @param {float} alpha            α 轴电压
 * @param {float} beta             β 轴电压
 * @param {uint32_t} PWMFullDutyCycle  PWM 计数满量程 (ARR)
 * @param {uint32_t} *tAout        A 相 CCR 输出
 * @param {uint32_t} *tBout        B 相 CCR 输出
 * @param {uint32_t} *tCout        C 相 CCR 输出
 * @param {uint32_t} *svm_sector   SVPWM 扇区号 (1~6)
 * @return {*}
 * @author: lkc
 */
void foc_svm(float alpha, float beta, uint32_t PWMFullDutyCycle,
				uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector) 
{
	uint32_t sector;

	/* 扇区判断 */
	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			//quadrant I
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 2;
			} else {
				sector = 1;
			}
		} else {
			//quadrant II
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 3;
			} else {
				sector = 2;
			}
		}
	} else {
		if (alpha >= 0.0f) {
			//quadrant IV5
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 5;
			} else {
				sector = 6;
			}
		} else {
			//quadrant III
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 4;
			} else {
				sector = 5;
			}
		}
	}

	// PWM timings
	uint32_t tA, tB, tC;

	switch (sector) {

	// sector 1-2
	case 1: {
		// Vector on-times
		uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		uint32_t t2 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tA = (PWMFullDutyCycle + t1 + t2) / 2;
		tB = tA - t1;
		tC = tB - t2;

		break;
	}

	// sector 2-3
	case 2: {
		// Vector on-times
		uint32_t t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tB = (PWMFullDutyCycle + t2 + t3) / 2;
		tA = tB - t3;
		tC = tA - t2;

		break;
	}

	// sector 3-4
	case 3: {
		// Vector on-times
		uint32_t t3 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tB = (PWMFullDutyCycle + t3 + t4) / 2;
		tC = tB - t3;
		tA = tC - t4;

		break;
	}

	// sector 4-5
	case 4: {
		// Vector on-times
		uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		uint32_t t5 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tC = (PWMFullDutyCycle + t4 + t5) / 2;
		tB = tC - t5;
		tA = tB - t4;

		break;
	}

	// sector 5-6
	case 5: {
		// Vector on-times
		uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
		uint32_t t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tC = (PWMFullDutyCycle + t5 + t6) / 2;
		tA = tC - t5;
		tB = tA - t6;

		break;
	}

	// sector 6-1
	case 6: {
		// Vector on-times
		uint32_t t6 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
		uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

		// PWM timings
		tA = (PWMFullDutyCycle + t6 + t1) / 2;
		tC = tA - t1;
		tB = tC - t6;

		break;
	}
	}

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;
	*svm_sector = sector;
}

/* VESC：mod = vdq / ((2/3)*Vbus) = 1.5 * vdq / Vbus */
#define MOTOR_FOC_SVM_VOLTAGE_SCALE 1.5f

/**
 * @description: 按 VESC mcpwm_foc 把 αβ 电压变成中点 ARR/2 的三路 CCR。
 * @detail: 先把电压换成调制比，限幅到线性区后交给 foc_svm；v_bus 非法时输出 50% 占空比。
 * @param {float} v_alpha   α 轴电压 (V)
 * @param {float} v_beta    β 轴电压 (V)
 * @param {float} v_bus     母线电压 (V)
 * @param {float} max_duty  最大调制占空比 (0~1，无量纲)
 * @param {uint32_t} period PWM 计数满量程 (ARR)
 * @param {uint32_t} *tAout A 相 CCR
 * @param {uint32_t} *tBout B 相 CCR
 * @param {uint32_t} *tCout C 相 CCR
 * @param {uint32_t} *svm_sector SVPWM 扇区号 (1~6)
 * @return {*}
 * @author: lkc
 */
void foc_svm_from_voltage(float v_alpha, float v_beta, float v_bus, float max_duty, uint32_t period,
                          uint32_t *tAout, uint32_t *tBout, uint32_t *tCout, uint32_t *svm_sector)
{
    float mod_alpha;
    float mod_beta;
    uint32_t mid;

    COMMON_ASSERT(tAout != NULL && tBout != NULL && tCout != NULL && svm_sector != NULL);

    mid = period / 2U;
    if (v_bus <= 0.0f || period == 0U) {
        *tAout = mid;
        *tBout = mid;
        *tCout = mid;
        *svm_sector = 1U;
        return;
    }

    if (max_duty <= 0.0f) {
        max_duty = 1.0f;
    } else if (max_duty > 1.0f) {
        max_duty = 1.0f;
    }

    mod_alpha = v_alpha * MOTOR_FOC_SVM_VOLTAGE_SCALE / v_bus;
    mod_beta = v_beta * MOTOR_FOC_SVM_VOLTAGE_SCALE / v_bus;
    (void)utils_saturate_vector_2d(&mod_alpha, &mod_beta, SQRT3_BY_2 * max_duty);
    foc_svm(mod_alpha, mod_beta, period, tAout, tBout, tCout, svm_sector);
}

/**
 * @description: 慧驱动 SVPWM，将 αβ 电压矢量转为三路 PWM 比较值。
 * @detail: 使用当前功率板在 3e5d564 已验证能稳定开环旋转的扇区映射。
 * @param {float} Valpha   α 轴电压 (V)
 * @param {float} Vbeta    β 轴电压 (V)
 * @param {float} Udc      母线电压 (V)
 * @param {uint32_t} Tpwm  PWM 计数满量程 (ARR)
 * @param {uint32_t} *pTcmp1  A 相 CCR 输出
 * @param {uint32_t} *pTcmp2  B 相 CCR 输出
 * @param {uint32_t} *pTcmp3  C 相 CCR 输出
 * @param {uint32_t} *pSector SVPWM 扇区号 (1~6)
 * @return {*}
 * @author: lkc
 */
void svm_hqd(float Valpha, float Vbeta, float Udc, uint32_t Tpwm, uint32_t *pTcmp1, uint32_t *pTcmp2, uint32_t *pTcmp3, uint32_t *pSector)
{
    float Vref1;
    float Vref2;
    float Vref3;
    float ta;
    float tb;
    float tc;
    float X;
    float Y;
    float Z;
    uint32_t Tcmp1 = 0U;
    uint32_t Tcmp2 = 0U;
    uint32_t Tcmp3 = 0U;
    uint32_t sector = 0U;
    float T1 = 0.0f;
    float T2 = 0.0f;

    COMMON_ASSERT(pTcmp1 != NULL && pTcmp2 != NULL && pTcmp3 != NULL && pSector != NULL);
    if (Udc <= 0.0f || Tpwm == 0U) {
        *pTcmp1 = 0U;
        *pTcmp2 = 0U;
        *pTcmp3 = 0U;
        *pSector = 0U;
        return;
    }

    Vref1 = Vbeta;
    Vref2 = (ONE_BY_SQRT3 * Valpha - Vbeta) / 2.0f;
    Vref3 = (-ONE_BY_SQRT3 * Valpha - Vbeta) / 2.0f;
    if (Vref1 > 0.0f) {
        sector = 1U;
    }
    if (Vref2 > 0.0f) {
        sector += 2U;
    }
    if (Vref3 > 0.0f) {
        sector += 4U;
    }

    /*
     * 下方时间合成会再除以 2；β 项若仍用旧系数，其实际调制增益
     * 只有 α 轴的一半。实机 300 RPM 的 Id/Iq 因此出现强烈二倍电角波动。
     * 将所有 β 分量系数同比乘 2，保留原扇区、整体极性和 A-C-B 映射。
     */
    X = TWO_BY_SQRT3 * Vbeta * (float)Tpwm / Udc;
    /*
     * 严格复现 3e5d564 的 ARMCC 数值语义：旧源码中的 3 / 2 是整数除法，
     * 实际系数为 1.0，而不是数学上直觉性的 1.5。这里先保持实机基线，
     * 等恢复旋转后再单独验证/更换标准 SVPWM。
     */
    Y = (float)Tpwm / Udc * (Valpha + ONE_BY_SQRT3 * Vbeta);
    Z = (float)Tpwm / Udc * (-Valpha + ONE_BY_SQRT3 * Vbeta);

    switch (sector) {
        case 1U: T1 = Z;  T2 = Y;  break;
        case 2U: T1 = Y;  T2 = -X; break;
        case 3U: T1 = -Z; T2 = X;  break;
        case 4U: T1 = -X; T2 = Z;  break;
        case 5U: T1 = X;  T2 = -Y; break;
        case 6U: T1 = -Y; T2 = -Z; break;
        default: break;
    }

    if ((T1 + T2) > (float)Tpwm) {
        float sum = T1 + T2;
        T1 = T1 / sum * (float)Tpwm;
        T2 = T2 / sum * (float)Tpwm;
    }

    ta = ((float)Tpwm - (T1 + T2)) / 4.0f;
    tb = ta + T1 / 2.0f;
    tc = tb + T2 / 2.0f;

    switch (sector) {
        case 1U: Tcmp1 = (uint32_t)tb; Tcmp2 = (uint32_t)ta; Tcmp3 = (uint32_t)tc; break;
        case 2U: Tcmp1 = (uint32_t)ta; Tcmp2 = (uint32_t)tc; Tcmp3 = (uint32_t)tb; break;
        case 3U: Tcmp1 = (uint32_t)ta; Tcmp2 = (uint32_t)tb; Tcmp3 = (uint32_t)tc; break;
        case 4U: Tcmp1 = (uint32_t)tc; Tcmp2 = (uint32_t)tb; Tcmp3 = (uint32_t)ta; break;
        case 5U: Tcmp1 = (uint32_t)tc; Tcmp2 = (uint32_t)ta; Tcmp3 = (uint32_t)tb; break;
        case 6U: Tcmp1 = (uint32_t)tb; Tcmp2 = (uint32_t)tc; Tcmp3 = (uint32_t)ta; break;
        default: break;
    }

    *pTcmp1 = Tcmp1;
    *pTcmp2 = Tcmp2;
    *pTcmp3 = Tcmp3;
    *pSector = sector;

    return;
}
#ifdef __cplusplus
}
#endif
