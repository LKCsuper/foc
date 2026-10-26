/* 停机对照用原始正余弦基准的实现：宏改名引入第三方算法源，不改动第三方文件 */
/* Includes ------------------------------------------------------------------*/
#include "motor_reference_trig.h"
#if MOTOR_CFG_REFERENCE_TRIG
#include <math.h>

#if defined(__CC_ARM)
#pragma O2
#endif

#ifdef M_PI
#undef M_PI
#endif
/* Private define ------------------------------------------------------------*/
#define M_PI motor_reference_pi
#define PI_2 motor_reference_pi_2
#define my_sin motor_reference_sin
#define my_cos motor_reference_cos
#define arctan motor_reference_arctan
#define arcsin motor_reference_arcsin
#define Q_rsqrt motor_reference_rsqrt
#include "../../../ThirtyLibrary/20261019WQFSBHXM/MyAlgorithm/HARDWARE/myalgorithm.c"
#undef M_PI
#undef PI_2
#undef my_sin
#undef my_cos
#undef arctan
#undef arcsin
#undef Q_rsqrt

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 只供停机对照，调用原始正余弦函数，不修改第三方文件。
 * @detail: 不做角度缩减或精度修正，terms 由基准限定为 3、5、7、9、12。禁止接入真实控制输出。
 * @param {float} angle 输入弧度
 * @param {int} terms 泰勒项数
 * @param {float *} sine 正弦输出
 * @param {float *} cosine 余弦输出
 * @return {void} 无
 */
void Motor_ReferenceTrigRaw(float angle, int terms, float *sine, float *cosine)
{
    *sine = motor_reference_sin(angle, terms);
    *cosine = motor_reference_cos(angle, terms);
}
#endif /* MOTOR_CFG_REFERENCE_TRIG */
