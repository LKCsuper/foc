/* 网页协议边界校验与序号/超时判定；调用者负责跨任务互斥。 */
#include "studio_protocol.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

uint16_t Studio_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffffU;
    size_t i;
    unsigned bit;
    for (i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8; bit++) crc = (uint16_t)((crc & 0x8000U) ? (crc << 1) ^ 0x1021U : crc << 1);
    }
    return crc;
}

static bool parse_u32(const char **cursor, uint32_t *out)
{
    uint32_t value = 0;
    const char *p = *cursor;
    if (*p < '0' || *p > '9') return false;
    while (*p >= '0' && *p <= '9') {
        unsigned digit = (unsigned)(*p++ - '0');
        if (value > (UINT32_MAX - digit) / 10U) return false;
        value = value * 10U + digit;
    }
    if (*p++ != ',') return false;
    *out = value;
    *cursor = p;
    return true;
}

bool Studio_ParseTarget(const char *line, studio_target_t *target)
{
    const char *star, *p;
    char *end;
    unsigned i, checksum = 0;
    studio_target_t candidate;
    float *values[3] = {&candidate.rpm, &candidate.iq, &candidate.id};
    if (line == NULL || target == NULL || strlen(line) > 159U || strncmp(line, "@C1,", 4U)) return false;
    star = strchr(line, '*');
    if (star == NULL || strlen(star + 1) != 4U) return false;
    for (i = 1; i <= 4; i++) {
        char c = star[i];
        unsigned digit;
        if (c >= '0' && c <= '9') digit = (unsigned)(c - '0');
        else if (c >= 'A' && c <= 'F') digit = (unsigned)(c - 'A') + 10U;
        else if (c >= 'a' && c <= 'f') digit = (unsigned)(c - 'a') + 10U;
        else return false;
        checksum = (checksum << 4) | digit;
    }
    if (Studio_Crc16((const uint8_t *)line, (size_t)(star - line)) != checksum) return false;
    p = line + 4;
    if (!parse_u32(&p, &candidate.token) || !parse_u32(&p, &candidate.sequence)) return false;
    for (i = 0; i < 3; i++) {
        if (*p == ' ' || *p == '\t') return false;
        *values[i] = strtof(p, &end);
        if (end == p || !isfinite(*values[i]) || (i < 2 ? *end != ',' : end != star)) return false;
        p = end + 1;
    }
    *target = candidate;
    return true;
}

size_t Studio_Encode(uint8_t type, uint16_t sequence, const uint8_t *payload, uint16_t length, uint8_t *output)
{
    uint16_t crc;
    if (output == NULL || length > 86U || (length != 0U && payload == NULL)) return 0U;
    output[0] = 'F'; output[1] = 'S'; output[2] = 1U; output[3] = type;
    output[4] = (uint8_t)length; output[5] = (uint8_t)(length >> 8);
    output[6] = (uint8_t)sequence; output[7] = (uint8_t)(sequence >> 8);
    if (length) memcpy(output + 8, payload, length);
    crc = Studio_Crc16(output, (size_t)length + 8U);
    output[8U + length] = (uint8_t)crc;
    output[9U + length] = (uint8_t)(crc >> 8);
    return (size_t)length + 10U;
}

void Studio_LeaseArm(studio_lease_t *lease, uint32_t now_us)
{
    lease->token++;
    if (lease->token == 0U) lease->token = 1U;
    lease->last_us = now_us;
    lease->last_sequence = 0U;
    lease->active = true;
    lease->expired = false;
    lease->have_sequence = false;
}

bool Studio_LeaseExpire(studio_lease_t *lease, uint32_t now_us)
{
    if (!lease->active || (uint32_t)(now_us - lease->last_us) < STUDIO_TIMEOUT_US) return false;
    lease->active = false;
    lease->expired = true;
    return true;
}

bool Studio_LeaseAccept(studio_lease_t *lease, const studio_target_t *target, uint32_t now_us)
{
    uint32_t difference = target->sequence - lease->last_sequence;
    if (Studio_LeaseExpire(lease, now_us) || !lease->active || target->token != lease->token) return false;
    if (lease->have_sequence && (difference == 0U || difference >= 0x80000000UL)) return false;
    lease->last_us = now_us;
    lease->last_sequence = target->sequence;
    lease->have_sequence = true;
    return true;
}
