#ifndef RFID_BEHAVIOR_H
#define RFID_BEHAVIOR_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t x, y, z;
} accel_sample_t;
void behavior_reset(void);
/* A result is available after 25 samples. Legacy warm-up result 0 counts as other. */
bool behavior_push(accel_sample_t sample, uint8_t *result);
#endif
