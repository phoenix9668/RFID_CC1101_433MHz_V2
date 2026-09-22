/* Text I/O is deliberately confined to the host runner, never firmware. */
#include "resampler.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    bool filtered = argc > 1 && strcmp(argv[1], "--filtered") == 0;
    resampler_t s; resampler_reset(&s); behavior_reset();
    uint64_t t, center; int x, y, z;
    unsigned count = 0;
    while (scanf("%" SCNu64 ",%d,%d,%d", &t, &x, &y, &z) == 4)
    {
        if (x < -32768 || x > 32767 || y < -32768 || y > 32767 || z < -32768 || z > 32767) return 2;
        accel_sample_t sample = {(int16_t)x, (int16_t)y, (int16_t)z}, out;
        if (filtered)
        {
            rfid_status_t r = resampler_push(&s, sample, t, &out, &center);
            if (r == RFID_INVALID) return 3;
            if (r != RFID_OK) continue;
            sample = out;
        }
        else center = t;
        uint8_t result = 0;
        bool ready = behavior_push(sample, &result);
        printf("%" PRIu64 ",%d,%d,%d,%d\n", center, sample.x, sample.y, sample.z, ready ? result : -1);
        ++count;
    }
    (void)count;
    return 0;
}
