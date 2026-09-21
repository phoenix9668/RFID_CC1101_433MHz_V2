#include "protocol.h"
#include <string.h>
uint32_t rfid_crc32(const void *data, size_t size)
{
    const uint8_t *p = data;
    uint32_t crc = UINT32_MAX;
    while (size--)
    {
        crc ^= *p++;
        for (unsigned i = 0; i < 8; ++i)
            crc = (crc >> 1) ^ ((0U - (crc & 1U)) & UINT32_C(0xedb88320));
    }
    return ~crc;
}
static void be16(uint8_t *p, uint16_t n)
{
    p[0] = n >> 8;
    p[1] = (uint8_t)n;
}
rfid_status_t protocol_encode(uint8_t *out, size_t capacity, const uint8_t id[6],
                              const uint8_t random[32], const rfid_history_t *state,
                              uint16_t battery)
{
    if (!out || !id || !random || !state || capacity < RFID_PAYLOAD_SIZE ||
        state->stage >= RFID_WINDOWS)
        return RFID_INVALID;
    memcpy(out, id, 6);
    memcpy(out + 6, random, 32);
    for (unsigned c = 0; c < RFID_CLASSES; ++c)
        for (unsigned w = 0; w < RFID_WINDOWS; ++w)
            be16(out + 38 + 24 * c + 2 * w, state->history[c][w]);
    out[182] = state->stage;
    be16(out + 183, battery);
    be16(out + 185, state->reset_count);
    uint32_t crc = rfid_crc32(out, 187);
    out[187] = crc >> 24;
    out[188] = crc >> 16;
    out[189] = crc >> 8;
    out[190] = (uint8_t)crc;
    return RFID_OK;
}
