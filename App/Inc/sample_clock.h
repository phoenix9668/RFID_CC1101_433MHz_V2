#ifndef RFID_SAMPLE_CLOCK_H
#define RFID_SAMPLE_CLOCK_H
#include "rfid_types.h"
typedef struct {
    uint32_t previous_ms, anchor_ms, observed_words, rate_millihz;
    uint64_t start_us, end_us;
    uint16_t words;
    bool started, ready;
} sample_clock_t;
void sample_clock_reset(sample_clock_t *c);
/* Each successful read drains the exact occupancy sampled at now_ms.
 * The first batch establishes an anchor and is discarded. A short read retains
 * the last long-span rate check. Input timing is reconstructed uniformly between
 * read snapshots, not measured per sample; bounded service jitter is assumed. */
rfid_status_t sample_clock_batch(sample_clock_t *c, uint32_t now_ms, uint16_t words);
uint64_t sample_clock_word_time(const sample_clock_t *c, unsigned word_index);
#endif
