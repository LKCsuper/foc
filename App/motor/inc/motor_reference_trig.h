#ifndef MOTOR_REFERENCE_TRIG_H
#define MOTOR_REFERENCE_TRIG_H

/**
 * @description: 只供停机对照，调用用户提供的原始正余弦函数，不修改第三方文件。
 * @detail: 不做角度缩减或精度修正，terms 由基准限定为 3、5、7、9、12。
 *          原始算法有输入范围/精度限制，禁止接入真实控制输出。
 * @param {float} angle 输入弧度
 * @param {int} terms 泰勒项数
 * @param {float *} sine 正弦输出
 * @param {float *} cosine 余弦输出
 * @return {void} 无
 */
void Motor_ReferenceTrigRaw(float angle, int terms, float *sine, float *cosine);

#endif
