/* MT6701 SSI/I2C 协议：CRC 校验、磁场状态与过速警告解码。 */
#include <assert.h>
#include <stdio.h>
#include "motor_mt6701_protocol.h"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_mt6701_frame_t frame = {0};
    assert(Motor_MT6701_DecodeSsi(0x8d1512U, &frame) == MOTOR_MT6701_FRAME_OK);
    assert(frame.angle == 0x2345U && frame.magnetic_status == 4U && !frame.overspeed_warning);
    for (unsigned bit = 0; bit < 24; ++bit) {
        frame.angle = 123U;
        assert(Motor_MT6701_DecodeSsi(0x8d1512U ^ (1U << bit), &frame) == MOTOR_MT6701_FRAME_CRC_ERROR);
        assert(frame.angle == 123U);
    }
    assert(Motor_MT6701_DecodeSsi(0x8d1606U, &frame) == MOTOR_MT6701_FRAME_OK);
    assert(frame.overspeed_warning);
    assert(Motor_MT6701_DecodeSsi(0x8d145dU, &frame) == MOTOR_MT6701_FRAME_MAGNETIC_ERROR);
    assert(frame.angle == 0x2345U && frame.magnetic_status == 1U);
    assert(Motor_MT6701_DecodeI2c(0x8dU, 0x14U) == 0x2345U);
    puts("MT6701 standalone protocol tests passed");
    return 0;
}
