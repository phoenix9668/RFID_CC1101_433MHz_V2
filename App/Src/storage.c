#include "storage.h"
#include "protocol.h"
#include <string.h>
enum
{
    RECORD_HISTORY = 1,
    RECORD_CHECKPOINT = 2,
    RECORD_VERSION = 1
};
#define COMMIT_MAGIC UINT32_C(0x38524649)
_Static_assert(STORAGE_START +
                       (STORAGE_HISTORY_SLOTS + STORAGE_CHECKPOINT_SLOTS) * STORAGE_RECORD_SIZE ==
                   STORAGE_END,
               "EEPROM layout");
static bool newer(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}
static void put16(uint8_t *p, uint16_t n)
{
    p[0] = (uint8_t)n;
    p[1] = n >> 8;
}
static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (uint16_t)p[1] << 8);
}
static void put32(uint8_t *p, uint32_t n)
{
    for (unsigned i = 0; i < 4; ++i)
        p[i] = n >> (8 * i);
}
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static bool read_record(storage_t *s, unsigned slot, uint8_t r[32])
{
    uint16_t base = STORAGE_START + slot * 32;
    for (unsigned i = 0; i < 8; ++i)
        put32(r + i * 4, s->read_word(s->context, base + i * 4));
    uint8_t kind = slot < STORAGE_HISTORY_SLOTS ? RECORD_HISTORY : RECORD_CHECKPOINT;
    return get32(r + 28) == COMMIT_MAGIC && r[22] == RECORD_VERSION && r[23] == kind &&
           r[20] < RFID_WINDOWS && r[21] == 0 && get16(r + 16) <= RFID_PERIOD_SECONDS &&
           get32(r + 24) == rfid_crc32(r, 24);
}
static rfid_status_t write_record(storage_t *s, const rfid_history_t *state, bool complete)
{
    if (!s || !state || !s->read_word || !s->write_word || state->stage >= RFID_WINDOWS ||
        state->elapsed > RFID_PERIOD_SECONDS)
        return RFID_INVALID;
    uint8_t r[32] = {0};
    put32(r, s->sequence + 1);
    for (unsigned c = 0; c < RFID_CLASSES; ++c)
        put16(r + 4 + c * 2, state->current[c]);
    put16(r + 16, state->elapsed);
    put16(r + 18, state->reset_count);
    r[20] = state->stage;
    r[22] = RECORD_VERSION;
    r[23] = complete ? RECORD_HISTORY : RECORD_CHECKPOINT;
    put32(r + 24, rfid_crc32(r, 24));
    put32(r + 28, COMMIT_MAGIC);
    unsigned slot = complete ? s->next_history : STORAGE_HISTORY_SLOTS + s->next_checkpoint;
    uint16_t base = STORAGE_START + slot * 32;
    /* Invalidate before reuse; the previous committed slot remains untouched. */
    rfid_status_t status = s->write_word(s->context, base + 28, 0);
    for (unsigned i = 0; status == RFID_OK && i < 8; ++i)
        status = s->write_word(s->context, base + i * 4, get32(r + i * 4));
    if (status != RFID_OK)
        return status;
    uint8_t verify[32];
    if (!read_record(s, slot, verify) || memcmp(r, verify, 32))
        return RFID_IO;
    s->sequence++;
    if (complete)
        s->next_history = (s->next_history + 1) % STORAGE_HISTORY_SLOTS;
    else
        s->next_checkpoint = (s->next_checkpoint + 1) % STORAGE_CHECKPOINT_SLOTS;
    return RFID_OK;
}
rfid_status_t storage_checkpoint(storage_t *s, const rfid_history_t *state)
{
    return write_record(s, state, false);
}
rfid_status_t storage_complete(storage_t *s, const rfid_history_t *state)
{
    return write_record(s, state, true);
}
rfid_status_t storage_restore(storage_t *s, rfid_history_t *state)
{
    if (!s || !state || !s->read_word || !s->write_word)
        return RFID_INVALID;
    memset(state, 0, sizeof(*state));
    s->sequence = 0;
    s->next_history = 0;
    s->next_checkpoint = 0;
    uint32_t slot_seq[RFID_WINDOWS] = {0}, hs = 0, cs = 0;
    bool present[RFID_WINDOWS] = {false}, found = false, have_h = false, have_c = false;
    uint8_t latest[32] = {0}, r[32];
    for (unsigned slot = 0; slot < STORAGE_HISTORY_SLOTS + STORAGE_CHECKPOINT_SLOTS; ++slot)
    {
        if (!read_record(s, slot, r))
            continue;
        uint32_t seq = get32(r);
        if (!found || newer(seq, s->sequence))
        {
            memcpy(latest, r, 32);
            s->sequence = seq;
            found = true;
        }
        if (slot < STORAGE_HISTORY_SLOTS)
        {
            unsigned stage = r[20];
            if (!present[stage] || newer(seq, slot_seq[stage]))
            {
                for (unsigned c = 0; c < RFID_CLASSES; ++c)
                    state->history[c][stage] = get16(r + 4 + c * 2);
                slot_seq[stage] = seq;
                present[stage] = true;
            }
            if (!have_h || newer(seq, hs))
            {
                hs = seq;
                have_h = true;
                s->next_history = (slot + 1) % STORAGE_HISTORY_SLOTS;
            }
        }
        else if (!have_c || newer(seq, cs))
        {
            cs = seq;
            have_c = true;
            s->next_checkpoint = (slot - STORAGE_HISTORY_SLOTS + 1) % STORAGE_CHECKPOINT_SLOTS;
        }
    }
    if (found && have_c)
    {
        state->stage = latest[20];
        state->reset_count = get16(latest + 18);
        if (latest[23] == RECORD_HISTORY)
            state->stage = (state->stage + 1) % RFID_WINDOWS;
        else
        {
            state->elapsed = get16(latest + 16);
            for (unsigned c = 0; c < RFID_CLASSES; ++c)
                state->current[c] = get16(latest + 4 + c * 2);
        }
        return RFID_OK;
    }
    /* Legacy identity stays at offsets 0 and 4 and is never written here. */
    static const uint16_t old_base[6] = {0x100, 0x130, 0x160, 0x190, 0x1c0, 0x200};
    uint32_t stage = s->read_word(s->context, 8), tick = s->read_word(s->context, 16);
    uint32_t resets = s->read_word(s->context, 12);
    state->reset_count = resets <= UINT16_MAX ? (uint16_t)resets : 0;
    state->stage = stage < RFID_WINDOWS ? (uint8_t)stage : 0;
    state->elapsed = stage < RFID_WINDOWS && tick < 120 ? (uint16_t)(tick * 10) : 0;
    bool valid[RFID_WINDOWS];
    for (unsigned w = 0; w < RFID_WINDOWS; ++w)
    {
        uint32_t total = 0;
        valid[w] = true;
        for (unsigned c = 0; c < RFID_CLASSES; ++c)
        {
            uint32_t value = s->read_word(s->context, old_base[c] + w * 4);
            if (value > 1500)
                valid[w] = false;
            else
                total += value;
            state->history[c][w] = value <= 1500 ? (uint16_t)value : 0;
        }
        valid[w] = valid[w] && total <= 1500;
        if (!valid[w])
            for (unsigned c = 0; c < RFID_CLASSES; ++c)
                state->history[c][w] = 0;
    }
    /* A corrupt class makes the entire legacy window unknown, not partial data. */
    if (!valid[state->stage])
        state->elapsed = 0;
    if (stage < RFID_WINDOWS && tick < 120 && valid[state->stage])
        for (unsigned c = 0; c < RFID_CLASSES; ++c)
            state->current[c] = state->history[c][state->stage];
    /* Import completed slots oldest first, then publish the current checkpoint.
       Interrupted imports remain reconstructible from untouched legacy storage. */
    for (unsigned k = 1; k <= RFID_WINDOWS; ++k)
    {
        rfid_history_t record = *state;
        record.stage = (state->stage + k) % RFID_WINDOWS;
        for (unsigned c = 0; c < RFID_CLASSES; ++c)
            record.current[c] = state->history[c][record.stage];
        rfid_status_t status = storage_complete(s, &record);
        if (status != RFID_OK)
            return status;
    }
    return storage_checkpoint(s, state);
}
