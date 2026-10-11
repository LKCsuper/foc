/* MT6701帧解码，不依赖HAL、任务、系统时钟或默认驱动实例。 */
#ifndef MOTOR_MT6701_PROTOCOL_H
#define MOTOR_MT6701_PROTOCOL_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    MOTOR_MT6701_FRAME_OK = 0,
    MOTOR_MT6701_FRAME_CRC_ERROR,
    MOTOR_MT6701_FRAME_MAGNETIC_ERROR
} motor_mt6701_frame_result_t;
typedef struct {
    uint16_t angle; /* 14位机械计数 */
    uint8_t magnetic_status;
    bool overspeed_warning; /* 沿用当前驱动策略：告警不单独拒绝角度 */
} motor_mt6701_frame_t;
/** 解码24位SSI帧；CRC错误时不改decoded，CRC正确时输出磁场状态供诊断。decoded不可为空。 */
motor_mt6701_frame_result_t Motor_MT6701_DecodeSsi(uint32_t frame, motor_mt6701_frame_t *decoded);
/** 解码依次从0x03、0x04读取的字节；传输有效性由调用者判定。 */
uint16_t Motor_MT6701_DecodeI2c(uint8_t msb, uint8_t lsb);
#ifdef __cplusplus
}
#endif
#endif
