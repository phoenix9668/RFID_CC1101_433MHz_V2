#ifndef RFID_BOARD_H
#define RFID_BOARD_H
#include "rfid_types.h"
typedef enum
{
    PIN_RADIO_CS,
    PIN_SENSOR_CS,
    PIN_RADIO_SO,
    PIN_GDO0,
    PIN_GDO2,
    PIN_RADIO_POWER,
    PIN_TX_EN,
    PIN_RX_EN,
    PIN_SENSOR_IRQ
} board_pin_t;
void board_pin_write(board_pin_t pin, bool high);
bool board_pin_read(board_pin_t pin);
void board_spi_enable(unsigned bus, bool enabled);
rfid_status_t board_spi_exchange(unsigned bus, uint8_t *bytes, size_t count);
uint32_t board_millis(void);
uint32_t board_rtc_millis(void);
uint32_t board_critical_enter(void);
void board_critical_exit(uint32_t saved);
void board_delay(uint32_t ms);
void board_radio_off(void);
void board_init(void);
void board_idle(bool allow_stop);
void board_watchdog(void);
uint32_t board_eeprom_read(void *context, uint16_t offset);
rfid_status_t board_eeprom_write(void *context, uint16_t offset, uint32_t value);
rfid_status_t board_battery(uint16_t *value);
void board_identity(uint8_t id[6], uint64_t *seed, uint64_t *stream);
void board_log(const char *message);
#endif
