#ifndef RFID_TYPES_H
#define RFID_TYPES_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
enum
{
    RFID_CLASSES = 6,
    RFID_WINDOWS = 12,
    RFID_PERIOD_SECONDS = 1200,
    RFID_PAYLOAD_SIZE = 191
};
typedef enum
{
    RFID_OK = 0,
    RFID_IO,
    RFID_TIMEOUT,
    RFID_INVALID,
    RFID_BUSY,
    RFID_OVERFLOW,
    RFID_EMPTY,
    RFID_CRC
} rfid_status_t;
typedef struct
{
    uint16_t history[RFID_CLASSES][RFID_WINDOWS];
    uint16_t current[RFID_CLASSES];
    uint16_t elapsed;
    uint16_t reset_count;
    uint8_t stage;
} rfid_history_t;
#endif
