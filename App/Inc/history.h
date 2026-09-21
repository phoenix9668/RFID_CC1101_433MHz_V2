#ifndef RFID_HISTORY_H
#define RFID_HISTORY_H
#include "rfid_types.h"
void history_count(rfid_history_t *s, uint8_t behavior);
void history_close(rfid_history_t *s);
#endif
