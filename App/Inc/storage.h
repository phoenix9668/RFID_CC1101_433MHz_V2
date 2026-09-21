#ifndef RFID_STORAGE_H
#define RFID_STORAGE_H
#include "rfid_types.h"
enum
{
    STORAGE_START = 0x240,
    STORAGE_RECORD_SIZE = 32,
    STORAGE_HISTORY_SLOTS = 16,
    STORAGE_CHECKPOINT_SLOTS = 30,
    STORAGE_END = 0x800
};
typedef struct
{
    void *context;
    uint32_t (*read_word)(void *, uint16_t offset);
    rfid_status_t (*write_word)(void *, uint16_t offset, uint32_t value);
    uint32_t sequence;
    uint8_t next_history, next_checkpoint;
} storage_t;
rfid_status_t storage_restore(storage_t *store, rfid_history_t *state);
rfid_status_t storage_checkpoint(storage_t *store, const rfid_history_t *state);
rfid_status_t storage_complete(storage_t *store, const rfid_history_t *state);
#endif
