#ifndef RFID_FIFO_PARSER_H
#define RFID_FIFO_PARSER_H
#include "behavior.h"
typedef struct
{
    accel_sample_t partial;
    uint8_t next_axis;
    uint32_t discarded;
} fifo_parser_t;
void fifo_parser_reset(fifo_parser_t *p);
bool fifo_parser_word(fifo_parser_t *p, uint16_t word, accel_sample_t *out);
#endif
