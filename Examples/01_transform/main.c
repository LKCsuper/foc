/*
 * @Description: 学习示例 01 — Clarke/Park/逆 Park/SVM 主机往返（复用生产 motor_foc）
 * @Author: lkc
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_foc.h"

/**
 * @description: 断言浮点接近期望值，失败则退出进程
 * @param {const char} *name  断言名称
 * @param {float} got         实际值
 * @param {float} want        期望值
 * @param {float} eps         容差
 * @return {*}
 * @author: lkc
 */
static void expect_near(const char *name, float got, float want, float eps)
{
    float err = fabsf(got - want);
    if (err > eps) {
        fprintf(stderr, "FAIL %s: got=%g want=%g eps=%g\n", name, (double)got, (double)want, (double)eps);
        exit(1);
    }
}

/**
 * @description: 示例入口：Clarke/Park/逆 Park 往返与 SVM 占空比范围检查
 * @return {int} 0=通过，非 0=失败
 * @author: lkc
 */
int main(void)
{
    float ia = 1.0f;
    float ib = -0.5f;
    float ic = -0.5f;
    float ialpha = 0.0f;
    float ibeta = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
    float a2 = 0.0f;
    float b2 = 0.0f;
    float ta = 0.0f;
    float tb = 0.0f;
    float tc = 0.0f;
    int svm_rc;

    clarke_transform(ia, ib, ic, &ialpha, &ibeta);
    park_transform(ialpha, ibeta, 0.0f, &id, &iq);
    expect_near("Id@0", id, 1.0f, 1e-3f);
    expect_near("Iq@0", iq, 0.0f, 1e-3f);

    inverse_park(id, iq, 0.0f, &a2, &b2);
    expect_near("alpha_rt", a2, ialpha, 1e-3f);
    expect_near("beta_rt", b2, ibeta, 1e-3f);

    park_transform(ialpha, ibeta, (float)(3.14159265358979323846 * 0.5), &id, &iq);
    expect_near("Id@pi/2", id, 0.0f, 2e-3f);
    expect_near("Iq@pi/2", iq, -1.0f, 2e-3f);

    svm_rc = svm(0.1f, 0.05f, &ta, &tb, &tc);
    if (svm_rc != 0) {
        fprintf(stderr, "FAIL svm: rc=%d\n", svm_rc);
        return 1;
    }
    if (ta < 0.0f || ta > 1.0f || tb < 0.0f || tb > 1.0f || tc < 0.0f || tc > 1.0f) {
        fprintf(stderr, "FAIL svm duty out of range\n");
        return 1;
    }

    printf("example_01_transform: ok\n");
    return 0;
}
