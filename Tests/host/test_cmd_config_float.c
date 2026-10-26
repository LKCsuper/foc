/* 截取 cmd_openloop.c 的 Cmd_ConfigFloat，用替身验证显示、拒绝非法数字、限幅、联动和提交失败路径。 */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "common_assert.h"
#include "cmd_openloop.h"

MOTOR_PARAM_S motor;
static motor_config_t stored;
static motor_result_t set_result = MOTOR_OK;
static int set_calls, applied_calls, capture_fail;
static char out[512];

/**
 * @description: 捕获或转发主机测试的格式化输出。
 * @param {const char *} format format 参数
 * @param {...} ... 与格式字符串匹配的可变参数
 * @return {void} 无
 */
static void host_printf(const char *format, ...)
{
    va_list args;
    size_t used = strlen(out);
    va_start(args, format);
    vsnprintf(out + used, sizeof(out) - used, format, args);
    va_end(args);
}
#define PRINTF(...) host_printf(__VA_ARGS__)

/**
 * @description: 主机测试替身：捕获当前配置快照，供命令修改后整体提交。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_config_t *} cfg 配置
 * @return {bool} 捕获成功返回 true；cfg 为空或读取失败返回 false
 */
bool Cmd_Ol_CaptureConfig(motor_config_t *cfg)
{
    if (capture_fail) return false;
    *cfg = stored;
    return true;
}
/**
 * @description: 主机测试替身：按配置提交结果打印失败原因；成功时不输出。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_result_t} result 输出结果
 * @return {void} 无
 */
void Cmd_Ol_PrintConfigResult(motor_result_t result) { if (result != MOTOR_OK) PRINTF("Invalid config\n"); }
/**
 * @description: 主机测试替身：校验并应用配置（不改校准与运行命令目标）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 电机实例
 * @param {const motor_config_t *} cfg 配置
 * @return {motor_result_t} 测试记录值 set_result
 */
motor_result_t Motor_SetConfig(MOTOR_PARAM_S *m, const motor_config_t *cfg)
{
    assert(m == &motor && cfg != NULL);
    set_calls++;
    if (set_result == MOTOR_OK) stored = *cfg;
    return set_result;
}
/**
 * @description: 检查测试所记录的输出应用结果。
 * @return {void} 无
 */
static void applied(void) { applied_calls++; }
/**
 * @description: 计算配对测试值的限幅结果。
 * @param {motor_config_t *} cfg 配置
 * @return {void} 无
 */
static void clamp_partner(motor_config_t *cfg)
{
    if (cfg->ol_min_rpm > cfg->ol_max_rpm) cfg->ol_min_rpm = cfg->ol_max_rpm;
}

#include "cmd_config_float_under_test.inc"

static const cmd_config_float_t boost = {offsetof(motor_config_t, ol_v_boost), CMD_CONFIG_CLAMP_MIN, 0.0f,
                                         "V_boost = %.2f V  (usage: olboost <volts>)\n", "V_boost set to %.2f V\n",
                                         NULL, applied};
static const cmd_config_float_t iq = {offsetof(motor_config_t, speed_iq_limit), CMD_CONFIG_CLAMP_ABS, 0.0f,
                                      "Iq = %.3f A\n", "Iq set to %.3f A\n", NULL, NULL};
static const cmd_config_float_t maxrpm = {offsetof(motor_config_t, ol_max_rpm), CMD_CONFIG_CLAMP_MIN, 1.0f,
                                          "Max RPM = %.0f\n", "Max RPM set to %.0f\n", clamp_partner, NULL};

/**
 * @description: 运行当前测试场景。
 * @param {const cmd_config_float_t *} spec spec 参数
 * @param {int} argc 命令参数个数
 * @param {const char *} arg arg 参数
 * @return {int} 本次计算或查询的结果
 */
static int run(const cmd_config_float_t *spec, int argc, const char *arg)
{
    char *argv[2] = {"cmd", (char *)arg};
    out[0] = '\0';
    return Cmd_ConfigFloat(spec, argc, argv);
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    memset(&stored, 0, sizeof(stored));
    stored.ol_v_boost = 0.5f;
    stored.ol_max_rpm = 800.0f;
    stored.ol_min_rpm = 300.0f;

    assert(run(&boost, 1, NULL) == 0 && set_calls == 0 && strcmp(out, "V_boost = 0.50 V  (usage: olboost <volts>)\n") == 0);
    assert(run(&boost, 2, "1.25") == 0 && stored.ol_v_boost == 1.25f && set_calls == 1 && applied_calls == 1);
    assert(strcmp(out, "V_boost set to 1.25 V\n") == 0);
    assert(run(&boost, 2, "abc") == -1 && stored.ol_v_boost == 1.25f && set_calls == 1);
    assert(strncmp(out, "Invalid number: abc\n", 20) == 0);
    assert(run(&boost, 2, "1.5x") == -1 && run(&boost, 2, "nan") == -1 && set_calls == 1);
    assert(run(&boost, 2, "-3") == 0 && stored.ol_v_boost == 0.0f);
    assert(run(&iq, 2, "-2.5") == 0 && stored.speed_iq_limit == 2.5f);
    assert(run(&maxrpm, 2, "0") == 0 && stored.ol_max_rpm == 1.0f && stored.ol_min_rpm == 1.0f);
    set_result = MOTOR_ERR_BUSY;
    applied_calls = 0;
    assert(run(&boost, 2, "2") == -1 && stored.ol_v_boost == 0.0f && applied_calls == 0);
    set_result = MOTOR_OK;
    capture_fail = 1;
    assert(run(&boost, 1, NULL) == -1 && strcmp(out, "Invalid config\n") == 0);
    printf("cmd config float: show, strict parse, clamp, adjust and rejection passed\n");
    return 0;
}
