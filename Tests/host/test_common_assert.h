/*
 * @Description: 主机测试断言辅助接口，用于验证必需指针契约的终止路径。
 */
#ifndef TEST_COMMON_ASSERT_H
#define TEST_COMMON_ASSERT_H
#include <assert.h>
#include <setjmp.h>
#include <signal.h>
#include <stdlib.h>

static jmp_buf common_assert_jump;
/**
 * @description: 捕获测试中的断言终止信号。
 * @param {int} code 兼容参数，本实现不使用
 * @return {void} 无
 */
static void common_assert_signal(int code)
{
    (void)code;
    longjmp(common_assert_jump, 1);
}
/* 实际执行失败路径，确认主机后端触发 SIGABRT；不把断言替换为空操作。 */
#define EXPECT_COMMON_ASSERT(expression) do { \
    void (*previous)(int) = signal(SIGABRT, common_assert_signal); \
    if (setjmp(common_assert_jump) == 0) { \
        expression; \
        signal(SIGABRT, previous); \
        exit(EXIT_FAILURE); \
    } else { \
        signal(SIGABRT, previous); \
    } \
} while (0)
#endif
