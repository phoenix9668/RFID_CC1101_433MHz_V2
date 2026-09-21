#ifndef RFID_SENSOR_H
#define RFID_SENSOR_H
#include "rfid_types.h"
rfid_status_t sensor_init(void);
rfid_status_t sensor_read_register(uint8_t address, uint8_t *value);
rfid_status_t sensor_fifo_entries(uint16_t *entries, bool *overrun);
rfid_status_t sensor_fifo_read(uint8_t *data, uint16_t length);
rfid_status_t sensor_fifo_restart(void);
void sensor_port_clear_error(void);
rfid_status_t sensor_port_error(void);
#endif
