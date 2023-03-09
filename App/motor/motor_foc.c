/*
 * @Description: 
 * @Date: 2023-02-18 23:29:37
 * @LastEditTime: 2023-03-04 17:02:20
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
inline void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta  = (Ib - Ic) * ONE_BY_SQRT3;
}

/**
 * @description: park变换
 * @detail: 
 * @param {float} Ialpha
 * @param {float} Ibeta
 * @param {float} Theta
 * @param {float} *Id
 * @param {float} *Iq
 * @return {*}
 * @author: lkc
 */
inline void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
    float s = sin_f32(Theta);
    float c = cos_f32(Theta);
    *Id =   Ialpha * c + Ibeta * s;
    *Iq = - Ialpha * s + Ibeta * c;
}

/**
 * @description: 
 * @detail: 
 * @param {float} mod_d
 * @param {float} mod_q
 * @param {float} Theta
 * @param {float} *mod_alpha
 * @param {float} *mod_beta
 * @return {*}
 * @author: lkc
 */
inline void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta)
{
    float s = sin_f32(Theta);
    float c = cos_f32(Theta);
    *mod_alpha = mod_d * c - mod_q * s;
    *mod_beta  = mod_d * s + mod_q * c;
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
inline int svm(float alpha, float beta, float* tA, float* tB, float* tC)
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

#ifdef __cplusplus
}
#endif
