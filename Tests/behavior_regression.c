#include "behavior.h"
#include "legacy/adxl362.h"
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif
uint8_t ErrorIndex;
static unsigned seconds;
static unsigned counts[6];
static void check_block(const accel_sample_t samples[150])
{
    for (unsigned i = 0; i < 150; ++i) {
        int16_t axes[3] = { samples[i].x, samples[i].y, samples[i].z };
        for (unsigned a = 0; a < 3; ++a) {
            uint16_t word = ((uint16_t)axes[a] & 0x0fffU) | (a << 14);
            fifo[i * 6 + a * 2] = (uint8_t)word;
            fifo[i * 6 + a * 2 + 1] = (uint8_t)(word >> 8);
        }
    }
    ADXL362FifoProcess();
    unsigned produced = 0;
    for (unsigned i = 0; i < 150; ++i) {
        uint8_t result;
        if (behavior_push(samples[i], &result)) {
            if (result != action_classify_array[produced]) {
                fprintf(stderr, "Mismatch second %u: legacy %u new %u\n",
                        seconds, action_classify_array[produced], result);
                exit(1);
            }
            printf("%u,%u,%u\n", seconds++, action_classify_array[produced], result);
            counts[result >= 1 && result <= 6 ? result - 1 : 5]++;
            produced++;
        }
    }
    if (produced != 6) abort();
}
int main(int argc, char **argv)
{
#ifdef _WIN32
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    (void)argv;
    accel_sample_t samples[150];
    behavior_reset();
    if (argc > 1) {
        uint32_t state = 123456789;
        for (unsigned b = 0; b < 2200; ++b) {
            for (unsigned i = 0; i < 150; ++i) {
                state = state * 1664525U + 1013904223U;
                samples[i].x = (int16_t)((int)(state & 4095) - 2048);
                samples[i].y = (int16_t)((int)((state >> 12) & 4095) - 2048);
                samples[i].z = (int16_t)((int)((state >> 20) & 4095) - 2048);
                if (b % 50 < 20) samples[i] = (accel_sample_t){30, 24, -333};
            }
            check_block(samples);
        }
    } else {
        while (fread(samples, sizeof(samples), 1, stdin) == 1) check_block(samples);
    }
    uint16_t old[6] = {action_classify.rest, action_classify.ingestion,
        action_classify.movement, action_classify.climb, action_classify.ruminate, action_classify.other};
    for (unsigned i = 0; i < 6; ++i)
        if ((uint16_t)counts[i] != old[i]) return 2;
    fprintf(stderr, "PASS: %u seconds; counts %u %u %u %u %u %u\n",
            seconds, counts[0], counts[1], counts[2], counts[3], counts[4], counts[5]);
    return 0;
}
