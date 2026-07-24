/* Includes ------------------------------------------------------------------*/
#include "rand_bytes_gen.h"
#include "adc.h"
#include <string.h>

/* xoshiro128++ 1.0 (David Blackman and Sebastiano Vigna, public domain,
 * https://prng.di.unimi.it/xoshiro128plusplus.c). Not cryptographically
 * secure, but this MCU has no hardware TRNG and RandomString is only ever
 * used as RF-packet filler/obfuscation bytes, not key material. */
static uint32_t s[4];

static inline uint32_t rotl(const uint32_t x, int k)
{
    return (x << k) | (x >> (32 - k));
}

static uint32_t xoshiro128pp_next(void)
{
    const uint32_t result = rotl(s[0] + s[3], 7) + s[0];
    const uint32_t t = s[1] << 9;

    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = rotl(s[3], 11);

    return result;
}

/* splitmix32, used only to whiten/expand the small entropy pool into
 * xoshiro's 128-bit state (standard practice recommended by xoshiro's
 * authors when seeding from a small/weak entropy source). */
static uint32_t splitmix32_next(uint32_t *state)
{
    uint32_t z = (*state += 0x9E3779B9u);
    z = (z ^ (z >> 16)) * 0x21F0AAADu;
    z = (z ^ (z >> 15)) * 0x735A2D97u;
    return z ^ (z >> 15);
}

/* Array that will be filled with random bytes */
uint8_t RandomString[32] = {0, };

/* RNG init function */
void RNG_Init(void)
{
    uint32_t seed = HAL_GetTick();
    seed ^= adc.avgValue;

    const uint32_t *uid = (const uint32_t *)UID_BASE;
    seed ^= uid[0] ^ uid[1] ^ uid[2];

    uint32_t sm_state = seed;
    for (int i = 0; i < 4; i++)
    {
        s[i] ^= splitmix32_next(&sm_state);
    }

    if ((s[0] | s[1] | s[2] | s[3]) == 0)
    {
        s[0] = 1; /* xoshiro128 requires non-all-zero state */
    }

    rfid_printf("rng seed=%08lx\n", (unsigned long)seed);
}

/* RNG gen function */
void RNG_Gen(void)
{
    for (uint8_t i = 0; i < sizeof(RandomString); i += 4)
    {
        uint32_t r = xoshiro128pp_next();
        memcpy(&RandomString[i], &r, sizeof(r));
    }

    for (uint16_t i = 0; i < sizeof(RandomString); i++)
    {
        rfid_printf("%02x ", RandomString[i]);
    }

    rfid_printf("\n");
}
