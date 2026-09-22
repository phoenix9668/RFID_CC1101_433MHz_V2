#include "sample_clock.h"
#include <string.h>
void sample_clock_reset(sample_clock_t *c) { memset(c, 0, sizeof(*c)); }
rfid_status_t sample_clock_batch(sample_clock_t *c, uint32_t now_ms, uint16_t words)
{
    if (!c || !words || words > 512) return RFID_INVALID;
    if (!c->started)
    {
        c->started = true;
        c->previous_ms = c->anchor_ms = now_ms;
        c->end_us = (uint64_t)now_ms * 1000;
        return RFID_BUSY;
    }
    uint32_t elapsed = now_ms - c->previous_ms;
    if (!elapsed || elapsed > 10000)
    {
        sample_clock_reset(c);
        return RFID_INVALID;
    }
    c->start_us = c->end_us;
    c->end_us += (uint64_t)elapsed * 1000;
    c->previous_ms = now_ms;
    c->words = words;
    c->observed_words += words;
    uint32_t span = now_ms - c->anchor_ms;
    if (span >= (c->ready ? 20000U : 2000U))
    {
        uint32_t rate = (uint32_t)((uint64_t)c->observed_words * 1000000 / (3 * span));
        /* Occupancy snapshots quantize time by one sample, especially at the
         * first short calibration interval. Do not reject a boundary-rate part. */
        uint32_t tolerance = 1000000U / span + 1;
        if (rate + tolerance < 32000 || rate > 65000 + tolerance)
        {
            sample_clock_reset(c);
            return RFID_INVALID;
        }
        c->rate_millihz = rate;
        c->ready = true;
        c->anchor_ms = now_ms;
        c->observed_words = 0;
    }
    return c->ready ? RFID_OK : RFID_BUSY;
}
uint64_t sample_clock_word_time(const sample_clock_t *c, unsigned word_index)
{
    return c->start_us + (c->end_us - c->start_us) * (word_index + 1) / c->words;
}
