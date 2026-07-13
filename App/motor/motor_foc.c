/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-10-14 17:08:15
 * @FilePath: \foc\App\motor\utils.c
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: clarke 变换
 * @detail: 
 * @param {float} Ia
 * @param {float} Ib
 * @param {float} Ic
 * @param {float} *Ialpha
 * @param {float} *Ibeta
 * @return {*}
 * @author: lkc
 */
void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
	/*
	 * VESC 在三电阻(HW_HAS_3_SHUNTS) 模式下使用完整 Clarke 变换。
	 * 它同时使用三相采样，如果三相上有同向的小偏置，它会在
	 * 变换中相互抵消。旧实现只用 Ia/Ib，导致 Ic 虽然被采集却不参与
	 * D/Q 电流计算，对低电流起步特别敏感。
	 */
#ifdef HW_HAS_3_SHUNTS
	*Ialpha = (2.0f / 3.0f) * Ia - (1.0f / 3.0f) * Ib - (1.0f / 3.0f) * Ic;
	*Ibeta = (Ib - Ic) * ONE_BY_SQRT3;
#else
	/* 两电阻硬件没有可信的 Ic，按 Ia/Ib 假设三相平衡。 */
	*Ialpha = Ia;
	*Ibeta = ONE_BY_SQRT3 * Ia + TWO_BY_SQRT3 * Ib;
#endif

	return;
}

void inverse_clarke(float Ialpha, float Ibeta, float *Ia, float *Ib, float *Ic)
{
    // *Ia =  Ialpha + 24.0f / 2;
    // *Ib =  Ibeta * SQRT3_BY_2 - Ialpha / 2 + 24.0 / 2;
    // *Ic =  - Ialpha / 2 - Ibeta * SQRT3_BY_2 + 24.0 / 2;

    *Ia =  Ialpha;
    *Ib =  Ibeta * SQRT3_BY_2 - Ialpha / 2;
    *Ic =  - Ialpha / 2 - Ibeta * SQRT3_BY_2;
	
	return;
}

/**
 * @description: park变换
 * @detail: 
 * @param {float} Ialpha
 * @param {float} Ibeta
 * @param {float} Theta 相位,例如编码器角度
 * @param {float} *Id
 * @param {float} *Iq
 * @return {*}
 * @author: lkc
 */
void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
	float s = sin_f32(Theta);
	float c = cos_f32(Theta);
	*Id =   Ialpha * c + Ibeta * s;
	*Iq = - Ialpha * s + Ibeta * c;
}

/**
 * @description: 
 * @detail: 
 * [uα]   [cos角度 -sin角度] [ud]
 * [uβ]	  [sin角度 cos角度]  [uq]
 * @param {float} mod_d
 * @param {float} mod_q
 * @param {float} Theta
 * @param {float} *mod_alpha
 * @param {float} *mod_beta
 * @return {*}
 * @author: lkc
 */
void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta)
{
	float s = sin_f32(Theta);
	float c = cos_f32(Theta);
	*mod_alpha = mod_d * c - mod_q * s;
	*mod_beta  = mod_d * s + mod_q * c;
	
	return;
}

/**
 * @description: svpwm
 * @detail: 
 * @param {float} alpha
 * @param {float} beta
 * @param {float*} tA
 * @param {float*} tB
 * @param {float*} tC
 * @return {*}
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
 * @brief svm Space vector modulation. Magnitude must not be larger than sqrt(3)/2, or 0.866 to avoid overmodulation.
 *        See https://github.com/vedderb/bldc/pull/372#issuecomment-962499623 for a full description.
 * @param alpha voltage
 * @param beta Park transformed and normalized voltage
 * @param PWMFullDutyCycle is the peak value of the PWM counter. 占空比
 * @param tAout PWM duty cycle phase A (0 = off all of the time, PWMFullDutyCycle = on all of the time)
 * @param tBout PWM duty cycle phase B
 * @param tCout PWM duty cycle phase C
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

/**
 * @description: 慧驱动svpwm
 * @detail: 
 * @param {float} Valpha
 * @param {float} Vbeta
 * @param {float} Udc
 * @param {uint32_t} Tpwm
 * @param {uint32_t} *pTcmp1
 * @param {uint32_t} *pTcmp2
 * @param {uint32_t} *pTcmp3
 * @param {uint32_t} *pSector
 * @return {*}
 * @author: lkc
 */
void svm_hqd(float Valpha, float Vbeta, float Udc, uint32_t Tpwm, uint32_t *pTcmp1, uint32_t *pTcmp2, uint32_t *pTcmp3, uint32_t *pSector)
{
    float va;
    float vb;
    float vc;
    float v_max;
    float v_min;
    float v_span;
    float scale = 1.0f;
    float v_common;
    float duty_a;
    float duty_b;
    float duty_c;
    uint32_t sector;

    if (Udc <= 0.0f || Tpwm == 0U) {
        *pTcmp1 = 0U;
        *pTcmp2 = 0U;
        *pTcmp3 = 0U;
        *pSector = 0U;
        return;
    }

    /* Alpha/beta voltage vector -> three phase-to-neutral voltages. */
    va = Valpha;
    vb = -0.5f * Valpha + SQRT3_BY_2 * Vbeta;
    vc = -0.5f * Valpha - SQRT3_BY_2 * Vbeta;

    v_max = fmaxf(va, fmaxf(vb, vc));
    v_min = fminf(va, fminf(vb, vc));
    v_span = v_max - v_min;
    if (v_span > Udc) {
        scale = Udc / v_span;
        va *= scale;
        vb *= scale;
        vc *= scale;
        v_max *= scale;
        v_min *= scale;
    }

    /* Min/max common-mode injection centres all duties around 50%. */
    v_common = -0.5f * (v_max + v_min);
    duty_a = 0.5f + (va + v_common) / Udc;
    duty_b = 0.5f + (vb + v_common) / Udc;
    duty_c = 0.5f + (vc + v_common) / Udc;
    utils_truncate_number(&duty_a, 0.0f, 1.0f);
    utils_truncate_number(&duty_b, 0.0f, 1.0f);
    utils_truncate_number(&duty_c, 0.0f, 1.0f);

    if (Vbeta >= 0.0f) {
        if (Valpha >= 0.0f) {
            sector = (ONE_BY_SQRT3 * Vbeta > Valpha) ? 2U : 1U;
        } else {
            sector = (-ONE_BY_SQRT3 * Vbeta > Valpha) ? 3U : 2U;
        }
    } else {
        if (Valpha >= 0.0f) {
            sector = (-ONE_BY_SQRT3 * Vbeta > Valpha) ? 5U : 6U;
        } else {
            sector = (ONE_BY_SQRT3 * Vbeta > Valpha) ? 4U : 5U;
        }
    }

    *pTcmp1 = (uint32_t)(duty_a * (float)Tpwm);
    *pTcmp2 = (uint32_t)(duty_b * (float)Tpwm);
    *pTcmp3 = (uint32_t)(duty_c * (float)Tpwm);
    *pSector = sector;

    return;
}


/**
 * @description: pll锁相环
 * @detail: 
 * @param:phase
 * @param:dt
 * @param:phase_var
 * @param:speed_var
 * @param:conf
 * @return {*}
 * @author: lkc
 */
void foc_pll_run(float phase, float dt, float *phase_var,
					float *speed_var, mc_configuration *conf) {
	UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var;
	utils_norm_angle_rad(&delta_theta);
	UTILS_NAN_ZERO(*speed_var);
	*phase_var += (*speed_var + conf->foc_pll_kp * delta_theta) * dt;
	utils_norm_angle_rad((float*)phase_var);
	*speed_var += conf->foc_pll_ki * delta_theta * dt;
}
#ifdef __cplusplus
}
#endif
