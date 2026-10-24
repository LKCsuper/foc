/* 网页调试协议：纯 C 编解码与控制租约，不依赖 MCU/RTOS。 */
#ifndef STUDIO_PROTOCOL_H
#define STUDIO_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define STUDIO_PERIOD_US 20000U
#define STUDIO_TIMEOUT_US 300000U
#define STUDIO_CONFIG_COUNT 43U
#define STUDIO_PARAM_COUNT 63U
typedef struct {
    uint32_t token;
    uint32_t sequence;
    float rpm;
    float iq;
    float id;
} studio_target_t;
typedef struct {
    uint32_t token;
    uint32_t last_us;
    uint32_t last_sequence;
    bool active;
    bool expired;
    bool have_sequence;
} studio_lease_t;

uint16_t Studio_Crc16(const uint8_t *data, size_t length);
/* @C1,token,seq,rpm,iq,id*CRC16；CRC 覆盖 @ 至 * 前，输入不含行结束符。 */
bool Studio_ParseTarget(const char *line, studio_target_t *target);
/* 输出 FS/version/type/lenLE/seqLE/payload/crcLE，最多 96 字节。 */
size_t Studio_Encode(uint8_t type, uint16_t sequence, const uint8_t *payload, uint16_t length, uint8_t *output);
void Studio_LeaseArm(studio_lease_t *lease, uint32_t now_us);
bool Studio_LeaseAccept(studio_lease_t *lease, const studio_target_t *target, uint32_t now_us);
bool Studio_LeaseExpire(studio_lease_t *lease, uint32_t now_us);
#endif
