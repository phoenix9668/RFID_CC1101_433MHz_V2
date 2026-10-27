/* Host replay of the RFID_BENCH_RAW_DUMP UART stream; used by
 * Tools/compare-fifo-dump.py. Reads the board's "d t n s rate" and "w ..."
 * lines from stdin, re-runs the FIFO parser, sample clock, resampler and
 * classifier in the order of app.c's process_fifo(), and prints the same
 * d/x/r/c lines. "--samples" adds one "s t_us x y z" line per complete sample
 * with its reconstructed time, i.e. a real recording at the 50 Hz setting. */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "behavior.h"
#include "fifo_parser.h"
#include "resampler.h"
#include "sample_clock.h"

static fifo_parser_t parser;
static resampler_t resampler;
static sample_clock_t timebase;
static uint16_t words[512];

static void reset_all(void)
{
    fifo_parser_reset(&parser);
    resampler_reset(&resampler);
    sample_clock_reset(&timebase);
    behavior_reset();
}

static void process(unsigned long t, unsigned n, bool samples)
{
    for (unsigned i = 0; i < n; ++i)
    {
        accel_sample_t sample, out;
        uint64_t center;
        uint32_t discarded = parser.discarded;
        bool complete = fifo_parser_word(&parser, words[i], &sample);
        if (parser.discarded != discarded)
        {
            printf("x %lu %u 1 0\n", t, i);
            behavior_reset();
            resampler_reset(&resampler);
        }
        if (!complete)
            continue;
        uint64_t time_us = sample_clock_word_time(&timebase, i);
        if (samples)
            printf("s %" PRIu64 " %d %d %d\n", time_us, sample.x, sample.y, sample.z);
        rfid_status_t r = resampler_push(&resampler, sample, time_us, &out, &center);
        if (r == RFID_INVALID)
        {
            printf("x %lu %u 2 0\n", t, i);
            behavior_reset();
            continue;
        }
        if (r != RFID_OK)
            continue;
        printf("r %lu %d %d %d\n", (unsigned long)(center / 1000), out.x, out.y, out.z);
        uint8_t result;
        if (behavior_push(out, &result))
            printf("c %lu %u 0 0\n", (unsigned long)((center + RESAMPLER_HALF_US) / 1000), result);
    }
}

int main(int argc, char **argv)
{
    bool samples = argc > 1 && strcmp(argv[1], "--samples") == 0;
    char line[256];
    bool pending = false;
    reset_all();
    for (;;)
    {
        if (!pending && !fgets(line, sizeof(line), stdin))
            break;
        pending = false;
        if (line[0] == '\n' || line[0] == '\r')
            continue;
        unsigned long t;
        long n, s, rate;
        if (sscanf(line, "d %lu %ld %ld %ld", &t, &n, &s, &rate) != 4 || n < 0 || n > 512)
        {
            fprintf(stderr, "unexpected line: %s", line);
            return 2;
        }
        bool want = false;
        if (s >= 100)
        {
            /* 100 overrun, 101/102 SPI error, 103 sensor re-init: everything restarts. */
            reset_all();
            printf("d %lu %ld %ld 0\n", t, n, s);
        }
        else
        {
            rfid_status_t timing = sample_clock_batch(&timebase, (uint32_t)t, (uint16_t)n);
            printf("d %lu %ld %d %lu\n", t, n, (int)timing, (unsigned long)timebase.rate_millihz);
            if (timing == RFID_OK)
                want = true;
            else
            {
                fifo_parser_reset(&parser);
                resampler_reset(&resampler);
                behavior_reset();
            }
        }
        /* The board prints the words only for batches it processes; tolerate extra ones. */
        long got = 0;
        while (fgets(line, sizeof(line), stdin))
        {
            if (line[0] != 'w')
            {
                pending = true;
                break;
            }
            const char *p = line + 1;
            unsigned value;
            int used;
            while (got < n && sscanf(p, " %x%n", &value, &used) == 1)
            {
                words[got++] = (uint16_t)value;
                p += used;
            }
        }
        if (!want)
            continue;
        if (got != n)
        {
            fprintf(stderr, "batch t=%lu: expected %ld words, got %ld\n", t, n, got);
            return 2;
        }
        process(t, (unsigned)n, samples);
    }
    return 0;
}
