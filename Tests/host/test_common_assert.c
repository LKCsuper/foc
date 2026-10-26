/* COMMON_ASSERT 宏：表达式只求值一次，且定义 NDEBUG 后失败检查仍不消失。 */
#include "common_assert.h"
#include "test_common_assert.h"
#include <stdio.h>

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    int evaluations = 0;
    COMMON_ASSERT(++evaluations == 1);
    if (evaluations != 1) return 1;
    EXPECT_COMMON_ASSERT(COMMON_ASSERT(0));
    /* 与 C assert 不同，定义 NDEBUG 后失败检查也不能消失。 */
    puts("COMMON_ASSERT: single evaluation and failure path passed");
    return 0;
}
