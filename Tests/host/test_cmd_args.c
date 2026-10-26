/* 命令参数严格解析：非法输入必须拒绝且不改写输出。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "cmd_args.h"

int main(void)
{
    float f = 7.0f;
    uint32_t u = 7U;

    assert(Cmd_ArgFloat("1.5", &f) && f == 1.5f);
    assert(Cmd_ArgFloat("-0.25", &f) && f == -0.25f);
    assert(Cmd_ArgFloat("  3", &f) && f == 3.0f);
    assert(Cmd_ArgFloat("1e3", &f) && f == 1000.0f);
    f = 7.0f;
    assert(!Cmd_ArgFloat("abc", &f) && f == 7.0f);
    assert(!Cmd_ArgFloat("1.5x", &f) && f == 7.0f);
    assert(!Cmd_ArgFloat("", &f) && f == 7.0f);
    assert(!Cmd_ArgFloat(NULL, &f) && f == 7.0f);
    assert(!Cmd_ArgFloat("nan", &f) && f == 7.0f);
    assert(!Cmd_ArgFloat("inf", &f) && f == 7.0f);
    assert(!Cmd_ArgFloat("1e60", &f) && f == 7.0f);

    assert(Cmd_ArgU32("0", &u) && u == 0U);
    assert(Cmd_ArgU32("4294967295", &u) && u == 4294967295U);
    assert(Cmd_ArgU32("0x10", &u) && u == 16U);
    u = 7U;
    assert(!Cmd_ArgU32("4294967296", &u) && u == 7U);
    assert(!Cmd_ArgU32("-1", &u) && u == 7U);
    assert(!Cmd_ArgU32("+1", &u) && u == 7U);
    assert(!Cmd_ArgU32("12a", &u) && u == 7U);
    assert(!Cmd_ArgU32("", &u) && u == 7U);
    assert(!Cmd_ArgU32(NULL, &u) && u == 7U);
    printf("cmd args: strict float/u32 parsing passed\n");
    return 0;
}
