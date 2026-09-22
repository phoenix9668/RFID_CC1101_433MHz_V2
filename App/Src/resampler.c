#include "resampler.h"
#include <limits.h>
#include <string.h>
#include "resampler_kernel.h"

void resampler_reset(resampler_t *s) { memset(s, 0, sizeof(*s)); }

static int32_t weight(uint64_t distance)
{
    if (distance >= RESAMPLER_HALF_US) return 0;
    unsigned i = (unsigned)(distance / 1000), fraction = (unsigned)(distance % 1000);
    return kernel[i] + ((int32_t)(kernel[i + 1] - kernel[i]) * (int32_t)fraction) / 1000;
}

static int16_t rounded(int64_t value, int32_t denominator)
{
    int64_t result = value >= 0 ? (value + denominator / 2) / denominator :
                                -((-value + denominator / 2) / denominator);
    if (result > INT16_MAX) result = INT16_MAX;
    if (result < INT16_MIN) result = INT16_MIN;
    return (int16_t)result;
}

rfid_status_t resampler_push(resampler_t *s, accel_sample_t sample, uint64_t time_us,
                            accel_sample_t *out, uint64_t *center_us)
{
    if (!s || !out || !center_us) return RFID_INVALID;
    if (s->started && (time_us <= s->last_input || time_us - s->last_input > 40000 ||
                      time_us - s->last_input < 10000))
    {
        resampler_reset(s);
        return RFID_INVALID;
    }
    if (!s->started)
    {
        /* Whole-second origin keeps subsequent groups of 25 on the RTC grid. */
        s->next_center = ((time_us + RESAMPLER_HALF_US + 999999) / 1000000) * 1000000;
        s->started = true;
    }
    s->last_input = time_us;
    s->samples[s->head] = sample;
    s->times[s->head] = time_us;
    s->head = (s->head + 1) % RESAMPLER_CAPACITY;
    if (s->count < RESAMPLER_CAPACITY) ++s->count;
    if (time_us < s->next_center + RESAMPLER_HALF_US) return RFID_BUSY;
    unsigned oldest = s->count == RESAMPLER_CAPACITY ? s->head : 0;
    if (s->times[oldest] > s->next_center - RESAMPLER_HALF_US + 40000)
    {
        resampler_reset(s);
        return RFID_INVALID;
    }
    int64_t sx = 0, sy = 0, sz = 0;
    int32_t total = 0;
    for (unsigned i = 0; i < s->count; ++i)
    {
        uint64_t t = s->times[i];
        uint64_t distance = t > s->next_center ? t - s->next_center : s->next_center - t;
        int32_t w = weight(distance);
        sx += (int64_t)w * s->samples[i].x;
        sy += (int64_t)w * s->samples[i].y;
        sz += (int64_t)w * s->samples[i].z;
        total += w;
    }
    if (total <= 0)
    {
        resampler_reset(s);
        return RFID_INVALID;
    }
    *out = (accel_sample_t){rounded(sx, total), rounded(sy, total), rounded(sz, total)};
    *center_us = s->next_center;
    s->next_center += RESAMPLER_STEP_US;
    return RFID_OK;
}
