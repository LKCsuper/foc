#ifndef TEST_COMMON_ASSERT_H
#define TEST_COMMON_ASSERT_H
#include <assert.h>
#include <setjmp.h>
#include <signal.h>
#include <stdlib.h>

static jmp_buf common_assert_jump;
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
