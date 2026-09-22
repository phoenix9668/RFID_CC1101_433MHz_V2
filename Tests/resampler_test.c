#include "resampler.h"
#include "sample_clock.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_rate(double rate, double frequency)
{
    resampler_t s; resampler_reset(&s);
    unsigned outputs = 0;
    double error = 0, energy = 0;
    uint64_t previous = 0;
    for (unsigned i = 0; i < (unsigned)(rate * 120); ++i)
    {
        uint64_t t = (uint64_t)(i * 1000000.0 / rate + 0.5);
        double v = 700 * sin(6.283185307179586 * frequency * i / rate);
        accel_sample_t in = {(int16_t)lround(v), 513, -32768}, out;
        uint64_t center;
        rfid_status_t r = resampler_push(&s, in, t, &out, &center);
        assert(r == RFID_OK || r == RFID_BUSY);
        if (r == RFID_OK)
        {
            assert(!outputs || center - previous == 40000);
            assert(t >= center + RESAMPLER_HALF_US);
            assert(out.y == 513 && out.z == -32768);
            double expected = 700 * sin(6.283185307179586 * frequency * center / 1000000.0);
            error += (out.x - expected) * (out.x - expected);
            energy += out.x * (double)out.x;
            previous = center; ++outputs;
        }
    }
    assert(outputs >= 2960 && outputs <= 2970);
    if (frequency <= 4) assert(sqrt(error / outputs) < 4);
    if (frequency >= 12.5) assert(sqrt(energy / outputs) < 5);
}

int main(void)
{
    const double rates[] = {32, 39.502107, 50, 57.5, 65};
    for (unsigned i = 0; i < sizeof(rates)/sizeof(rates[0]); ++i)
    {
        test_rate(rates[i], 2);
        test_rate(rates[i], 4);
        test_rate(rates[i], 12.5);
        test_rate(rates[i], 14);
    }
    resampler_t s; resampler_reset(&s);
    accel_sample_t out; uint64_t t;
    assert(resampler_push(&s, (accel_sample_t){0}, 0, &out, &t) == RFID_BUSY);
    assert(resampler_push(&s, (accel_sample_t){0}, 0, &out, &t) == RFID_INVALID);
    assert(!s.started);
    assert(resampler_push(&s, (accel_sample_t){0}, 0, &out, &t) == RFID_BUSY);
    assert(resampler_push(&s, (accel_sample_t){0}, 200000, &out, &t) == RFID_INVALID);
    sample_clock_t c; sample_clock_reset(&c);
    assert(sample_clock_batch(&c, UINT32_MAX-999, 450) == RFID_BUSY);
    assert(sample_clock_batch(&c, 2000, 450) == RFID_OK);
    assert(c.rate_millihz == 50000 && c.end_us > (uint64_t)UINT32_MAX*1000);
    assert(sample_clock_word_time(&c, 449) == c.end_us);
    assert(sample_clock_batch(&c, 2000, 3) == RFID_INVALID);
    sample_clock_reset(&c);
    assert(sample_clock_batch(&c, 0, 450) == RFID_BUSY);
    assert(sample_clock_batch(&c, 7600, 450) == RFID_INVALID);
    puts("PASS: DC, signed limits, 32..65 Hz, exact 40ms grid, passband, alias rejection, gaps, RTC wrap");
    return 0;
}
