/* Event-level timing regression for app.c with a modelled MCU processing time.
 *
 * app_test.c steps an exact clock in 10 ms and processes in zero time. This
 * test reproduces what hid the short-batch fault on the bench: the RTC's
 * 1/256 s resolution, 1/2048 s wakeup ticks, the FIFO watermark edge, STOP
 * wake latency, EEPROM word writes of about 6 ms and the time the MCU spends
 * per drained word (measured on V3.8 on 2026-10-26: 0.374 ms/word without
 * flash prefetch, 0.295 ms/word with it), during which the sensor keeps
 * filling its FIFO. The sensor rate is swept across the accepted 32..65 Hz
 * band. Any resampler or sample-clock reset, FIFO overrun, or shortfall of
 * classifications fails the sweep. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app.h"
#include "behavior.h"
#include "board.h"
#include "radio.h"
#include "resampler.h"
#include "sample_clock.h"
#include "sensor.h"

static double rate_hz, ms_per_word;
static const double spi_fixed_us = 60, spi_word_us = 8.5, eeprom_write_us = 6000;
static const double wake_us = 1500, poll_us = 20;

static uint64_t now_us, phase_us, produced;
static uint16_t fifo[512];
static unsigned fifo_count;
static bool fifo_overrun;
static uint32_t eeprom[512];
static unsigned long eeprom_writes, words_read, reads;
static unsigned long resampler_invalid, clock_invalid, classifications, overruns;
static double last_invalid_gap_ms;

rfid_status_t sim_resampler_push(resampler_t *s, accel_sample_t sample, uint64_t t,
                                 accel_sample_t *out, uint64_t *center)
{
    uint64_t previous = s->last_input;
    bool started = s->started;
    rfid_status_t r = resampler_push(s, sample, t, out, center);
    if (r == RFID_INVALID)
    {
        ++resampler_invalid;
        if (started)
            last_invalid_gap_ms = (double)(int64_t)(t - previous) / 1000.0;
    }
    return r;
}
rfid_status_t sim_sample_clock_batch(sample_clock_t *c, uint32_t now_ms, uint16_t words)
{
    rfid_status_t r = sample_clock_batch(c, now_ms, words);
    if (r == RFID_INVALID)
        ++clock_invalid;
    return r;
}
bool sim_behavior_push(accel_sample_t sample, uint8_t *result)
{
    bool ready = behavior_push(sample, result);
    if (ready)
        ++classifications;
    return ready;
}

static uint64_t sample_time(uint64_t k) { return phase_us + (uint64_t)((double)k * 1e6 / rate_hz); }
static void produce_until(uint64_t t)
{
    while (sample_time(produced) <= t)
    {
        unsigned before = fifo_count;
        if (fifo_count + 3 > 512)
        {
            memmove(fifo, fifo + 3, (fifo_count - 3) * 2u); /* stream mode keeps the newest */
            fifo_count -= 3;
            fifo_overrun = true;
            ++overruns;
        }
        int16_t v[3] = {(int16_t)((int)(produced % 7) - 3), 120, -1000};
        for (unsigned a = 0; a < 3; ++a)
            fifo[fifo_count++] = (uint16_t)(a << 14 | ((uint16_t)v[a] & 0x3fff));
        ++produced;
        if (before < 450 && fifo_count >= 450)
            app_signal(APP_EVENT_FIFO);
    }
}
static void advance(uint64_t t)
{
    now_us = t;
    produce_until(t);
}

/* RTC sub-second resolution is 1/256 s, as in board_rtc_millis(). */
uint32_t board_rtc_millis(void)
{
    uint64_t seconds = now_us / 1000000u;
    uint32_t sub = (uint32_t)((now_us % 1000000u) * 256u / 1000000u);
    return (uint32_t)(seconds * 1000u + sub * 1000u / 256u);
}
uint32_t board_millis(void) { return (uint32_t)(now_us / 1000u); }
uint32_t board_critical_enter(void) { return 0; }
void board_critical_exit(uint32_t saved) { (void)saved; }
void board_watchdog(void) {}
void board_log(const char *message) { (void)message; }
uint32_t board_eeprom_read(void *context, uint16_t offset)
{
    (void)context;
    return eeprom[offset / 4];
}
rfid_status_t board_eeprom_write(void *context, uint16_t offset, uint32_t value)
{
    (void)context;
    if (eeprom[offset / 4] != value)
        ++eeprom_writes;
    eeprom[offset / 4] = value;
    return RFID_OK;
}
rfid_status_t board_battery(uint16_t *value)
{
    *value = 0x765;
    return RFID_OK;
}
void board_identity(uint8_t id[6], uint64_t *seed, uint64_t *stream)
{
    memset(id, 0x55, 6);
    *seed = 123;
    *stream = 456;
}
rfid_status_t sensor_init(void)
{
    fifo_count = 0;
    return RFID_OK;
}
rfid_status_t sensor_fifo_restart(void)
{
    fifo_count = 0;
    return RFID_OK;
}
rfid_status_t sensor_fifo_entries(uint16_t *entries, bool *overrun)
{
    produce_until(now_us);
    *entries = (uint16_t)fifo_count;
    *overrun = fifo_overrun;
    fifo_overrun = false;
    return RFID_OK;
}
rfid_status_t sensor_fifo_read(uint8_t *data, uint16_t length)
{
    unsigned words = length / 2u;
    memcpy(data, fifo, length);
    memmove(fifo, fifo + words, (fifo_count - words) * 2u);
    fifo_count -= words;
    words_read += words;
    ++reads;
    return RFID_OK;
}
rfid_status_t radio_begin(const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
    return RFID_OK;
}
rfid_status_t radio_poll(void) { return RFID_OK; }

typedef struct
{
    unsigned long resampler_invalid, clock_invalid, overruns, classifications, leftover_words;
    double last_gap_ms;
} result_t;

/* Main loop as entry.c/board_idle(): RTC wakeup ticks of 1/2048 s and the
 * rising-edge watermark interrupt; processing time advances the clock. */
static result_t run(double rate, double seconds, double phase_fraction)
{
    rate_hz = rate;
    memset(eeprom, 0xff, sizeof(eeprom));
    eeprom[0] = 0x01020304u;
    eeprom[1] = 0x05060000u;
    now_us = 1000000u;
    phase_us = now_us + (uint64_t)(phase_fraction * 1e6 / rate);
    produced = fifo_count = 0;
    fifo_overrun = false;
    eeprom_writes = words_read = reads = 0;
    resampler_invalid = clock_invalid = classifications = overruns = 0;
    last_invalid_gap_ms = 0;
    app_init();
    uint64_t end = now_us + (uint64_t)(seconds * 1e6);
    while (now_us < end)
    {
        unsigned long w0 = words_read, e0 = eeprom_writes, r0 = reads;
        bool allow = app_poll();
        double busy = poll_us + (reads != r0 ? spi_fixed_us + (words_read - w0) * spi_word_us : 30) +
                      (words_read - w0) * ms_per_word * 1000.0 + (eeprom_writes - e0) * eeprom_write_us;
        advance(now_us + (uint64_t)busy);
        if (!allow)
        {
            advance(now_us + 1000u); /* WFI until the 1 ms SysTick */
            continue;
        }
        if (app_events_pending())
            continue;
        uint32_t delay = app_wakeup_delay_ms();
        if (!delay)
            continue;
        uint32_t ticks = (delay * 2048u + 999u) / 1000u;
        uint64_t wake = now_us + (uint64_t)ticks * 1000000u / 2048u, edge = UINT64_MAX;
        if (fifo_count < 450)
            edge = sample_time(produced + (450u - fifo_count + 2u) / 3u - 1u);
        if (edge < wake)
            advance(edge);
        else
        {
            advance(wake);
            app_signal(APP_EVENT_RTC);
        }
        advance(now_us + (uint64_t)wake_us);
    }
    result_t r = {resampler_invalid, clock_invalid, overruns, classifications,
                  (unsigned long)(produced * 3 - words_read), last_invalid_gap_ms};
    return r;
}

static bool acceptable(const result_t *r, double seconds)
{
    /* One classification per second after about 30 s of warm-up; the sensor
     * FIFO may legitimately hold a deferred partial batch at the end. */
    return !r->resampler_invalid && !r->clock_invalid && !r->overruns &&
           r->classifications >= (unsigned long)(seconds - 40) && r->leftover_words <= 512;
}

int main(int argc, char **argv)
{
    if (argc == 7 && !strcmp(argv[1], "sweep"))
    {
        double lo = atof(argv[2]), hi = atof(argv[3]), step = atof(argv[4]), seconds = atof(argv[5]);
        ms_per_word = atof(argv[6]);
        unsigned failures = 0, rates = 0;
        for (double rate = lo; rate <= hi + 1e-9; rate += step, ++rates)
        {
            result_t r = run(rate, seconds, 0.37);
            if (!acceptable(&r, seconds))
            {
                ++failures;
                printf("FAIL rate=%.3f Hz resampler_resets=%lu clock_resets=%lu overruns=%lu "
                       "classifications=%lu leftover_words=%lu last_gap_ms=%.2f\n",
                       rate, r.resampler_invalid, r.clock_invalid, r.overruns, r.classifications,
                       r.leftover_words, r.last_gap_ms);
            }
        }
        if (failures)
        {
            printf("%u of %u rates failed at %.3f ms/word\n", failures, rates, ms_per_word);
            return 1;
        }
        printf("PASS: %u rates %.2f..%.2f Hz, %.0f s each, %.3f ms/word: no resets, no overruns\n",
               rates, lo, hi, seconds, ms_per_word);
        return 0;
    }
    if (argc == 4)
    {
        ms_per_word = atof(argv[2]);
        result_t r = run(atof(argv[1]), atof(argv[3]), 0.37);
        printf("rate=%s Hz ms_per_word=%s seconds=%s resampler_resets=%lu clock_resets=%lu overruns=%lu "
               "classifications=%lu leftover_words=%lu last_gap_ms=%.2f\n",
               argv[1], argv[2], argv[3], r.resampler_invalid, r.clock_invalid, r.overruns,
               r.classifications, r.leftover_words, r.last_gap_ms);
        return acceptable(&r, atof(argv[3])) ? 0 : 1;
    }
    fprintf(stderr, "usage: app_timing_test <rate_hz> <ms_per_word> <seconds>\n"
                    "       app_timing_test sweep <lo_hz> <hi_hz> <step_hz> <seconds> <ms_per_word>\n");
    return 2;
}
