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
#if RFID_SENSOR_ODR_TEST
rfid_status_t sensor_data_ready_begin(void);
rfid_status_t sensor_data_ready_begin_rate(uint16_t nominal_hz);
rfid_status_t sensor_data_ready_end(void);
#endif
#endif
