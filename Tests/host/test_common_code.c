#include <assert.h>
#include <stdio.h>

#include "ringbuf_test.h"
#include "seqlock_test.h"

int main(void)
{
    uint32_t seqlock_fails = seqlock_test_run();
    uint32_t ringbuf_fails = ringbuf_test_run();

    fprintf(stderr, "seqlock fails=%lu ringbuf fails=%lu\n",
            (unsigned long)seqlock_fails, (unsigned long)ringbuf_fails);
    assert(seqlock_fails == 0U);
    assert(ringbuf_fails == 0U);
    printf("test_common_code: pass\n");
    return 0;
}
