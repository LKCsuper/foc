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

/**
 * @description: 计算 CRC-16/CCITT 校验，用于网页帧完整性检查。
 * @param {const uint8_t *} data  数据首地址，length 为 0 时可为空
 * @param {size_t} length         数据长度，单位字节
 * @return {uint16_t} 16 位 CRC
 */
uint16_t Studio_Crc16(const uint8_t *data, size_t length);
/**
 * @description: @C1,token,seq,rpm,iq,id*CRC16；CRC 覆盖 @ 至 * 前，输入不含行结束符。
 * @param {const char *} line      输入行，须以 @C1, 开头、以 4 位十六进制 CRC 结尾
 * @param {studio_target_t *} target  输出目标，不可为空
 * @return {bool} 解析且校验通过返回 true
 */
bool Studio_ParseTarget(const char *line, studio_target_t *target);
/**
 * @description: 输出 FS/version/type/lenLE/seqLE/payload/crcLE，最多 96 字节。
 * @param {uint8_t} type        帧类型
 * @param {uint16_t} sequence   帧序号
 * @param {const uint8_t *} payload  负载，length 为 0 时可为空
 * @param {uint16_t} length     负载长度，上限 86 字节
 * @param {uint8_t *} output    输出缓冲，不可为空
 * @return {size_t} 完整帧长度；参数非法返回 0
 */
size_t Studio_Encode(uint8_t type, uint16_t sequence, const uint8_t *payload, uint16_t length, uint8_t *output);
/**
 * @description: 续签控制租约：token 自增并复位序号与超时状态。
 * @param {studio_lease_t *} lease  租约，不可为空
 * @param {uint32_t} now_us        当前时间戳，单位 us
 * @return {void} 无
 */
void Studio_LeaseArm(studio_lease_t *lease, uint32_t now_us);
/**
 * @description: 接受新的目标帧：校验 token 与序号单调性，成功后刷新租约时间。
 * @param {studio_lease_t *} lease      租约，不可为空
 * @param {const studio_target_t *} target  目标帧，不可为空
 * @param {uint32_t} now_us            当前时间戳，单位 us
 * @return {bool} true=目标被接受
 */
bool Studio_LeaseAccept(studio_lease_t *lease, const studio_target_t *target, uint32_t now_us);
/**
 * @description: 判定租约是否超时；超时后置 active=false、expired=true。
 * @param {studio_lease_t *} lease  租约，不可为空
 * @param {uint32_t} now_us        当前时间戳，单位 us
 * @return {bool} true=本次判定触发超时
 */
bool Studio_LeaseExpire(studio_lease_t *lease, uint32_t now_us);
#endif
