#include "common_assert.h"
#include "test_common_assert.h"
#include <stdio.h>

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
