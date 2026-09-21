#ifndef RFID_PROTOCOL_H
#define RFID_PROTOCOL_H
#include "rfid_types.h"
uint32_t rfid_crc32(const void *data, size_t size);
rfid_status_t protocol_encode(uint8_t *out, size_t capacity, const uint8_t id[6],
                              const uint8_t random[32], const rfid_history_t *state,
                              uint16_t battery);
#endif
