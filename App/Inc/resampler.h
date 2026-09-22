#ifndef RFID_RESAMPLER_H
#define RFID_RESAMPLER_H
#include "behavior.h"
#include "rfid_types.h"
#include <stdint.h>

#define RESAMPLER_CAPACITY 48
#define RESAMPLER_HALF_US 320000U
#define RESAMPLER_STEP_US 40000U
typedef struct {
    accel_sample_t samples[RESAMPLER_CAPACITY];
    uint64_t times[RESAMPLER_CAPACITY];
    uint64_t next_center, last_input;
    unsigned head, count;
    bool started;
} resampler_t;
void resampler_reset(resampler_t *s);
/* Uniform 32..65 Hz input; timestamps in monotonic microseconds. No extrapolation.
 * RFID_BUSY: warmup/no output, RFID_OK: one output, RFID_INVALID: discontinuity.
 * center_us is signal time; availability is center_us + RESAMPLER_HALF_US. */
rfid_status_t resampler_push(resampler_t *s, accel_sample_t sample, uint64_t time_us,
                            accel_sample_t *out, uint64_t *center_us);
#endif
