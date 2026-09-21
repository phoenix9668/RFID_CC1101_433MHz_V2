#ifndef RFID_RADIO_H
#define RFID_RADIO_H
#include "rfid_types.h"
rfid_status_t radio_begin(const uint8_t *data, size_t length);
rfid_status_t radio_poll(void);
void radio_abort(void);
rfid_status_t radio_receive(uint8_t *data, size_t capacity, size_t *length, uint32_t timeout_ms);
rfid_status_t radio_read_status(uint8_t address, uint8_t *value);
uint8_t radio_config_byte(unsigned index);
#endif
