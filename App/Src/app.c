#include "app.h"
#include "board.h"
#include "sensor.h"
#include "radio.h"
#include "storage.h"
#include "history.h"
#include "protocol.h"
#include "fifo_parser.h"
#include "pcg_basic.h"
#include <string.h>
#include <stdio.h>
static volatile uint32_t events;
static rfid_history_t history;
static storage_t store = {.read_word = board_eeprom_read, .write_word = board_eeprom_write};
static fifo_parser_t parser;
static pcg32_random_t rng;
static uint8_t fifo_bytes[1024], payload[RFID_PAYLOAD_SIZE], identity[6];
static uint32_t period_start, next_save, next_sensor_retry, next_report;
static uint32_t sensor_errors, radio_errors, storage_errors;
static bool sensor_ready, report_pending, transmitting, storage_ready;
static uint16_t battery;
uint32_t app_wakeup_delay_ms(void)
{
    uint32_t now = board_rtc_millis();
    uint32_t delay = 10000;
    const uint32_t deadlines[] = {next_save, period_start + RFID_PERIOD_SECONDS * 1000,
                                  sensor_ready ? now + 10000 : next_sensor_retry};
    for (unsigned i = 0; i < sizeof(deadlines) / sizeof(deadlines[0]); ++i)
    {
        int32_t remaining = (int32_t)(deadlines[i] - now);
        if (remaining <= 0)
            return 0;
        if ((uint32_t)remaining < delay)
            delay = (uint32_t)remaining;
    }
    return delay;
}
void app_signal(uint32_t flags)
{
    uint32_t saved = board_critical_enter();
    events |= flags;
    board_critical_exit(saved);
}
bool app_events_pending(void)
{
    return events != 0;
}
void app_snapshot(rfid_history_t *out)
{
    if (out)
        *out = history;
}
static uint32_t take_events(void)
{
    uint32_t primask = board_critical_enter();
    uint32_t flags = events;
    events = 0;
    board_critical_exit(primask);
    return flags;
}
static void log_state(const char *event)
{
#if RFID_DIAGNOSTICS
    char line[160];
    snprintf(line, sizeof(line), "%s stage=%u elapsed=%u reset=%u err=%lu/%lu/%lu\r\n", event,
             history.stage, history.elapsed, history.reset_count, (unsigned long)sensor_errors,
             (unsigned long)radio_errors, (unsigned long)storage_errors);
    board_log(line);
#else
    (void)event;
#endif
}
static void advance_to(uint32_t time)
{
    while ((int32_t)(time - period_start) >= RFID_PERIOD_SECONDS * 1000)
    {
        history.elapsed = RFID_PERIOD_SECONDS;
        if (storage_ready && storage_complete(&store, &history) != RFID_OK)
        {
            storage_errors++;
            storage_ready = false;
        }
        history_close(&history);
        period_start += RFID_PERIOD_SECONDS * 1000;
        uint8_t random[32];
        for (unsigned i = 0; i < 32; i += 4)
        {
            uint32_t n = pcg32_random_r(&rng);
            for (unsigned j = 0; j < 4; ++j)
                random[i + j] = n >> (j * 8);
        }
        if (board_battery(&battery) != RFID_OK)
            board_log("ADC read failed; retaining previous value\r\n");
        if (!report_pending && !transmitting &&
            protocol_encode(payload, sizeof(payload), identity, random, &history, battery) ==
                RFID_OK)
        {
            report_pending = true;
            next_report = board_millis() + pcg32_boundedrand_r(&rng, 100);
        }
        log_state("window");
    }
    if ((int32_t)(time - period_start) >= 0)
        history.elapsed = (time - period_start) / 1000;
}
static void process_fifo(uint32_t now, bool drain)
{
    uint16_t entries;
    bool overrun;
    rfid_status_t status = sensor_fifo_entries(&entries, &overrun);
    if (status != RFID_OK)
    {
        sensor_ready = false;
        sensor_errors++;
        return;
    }
    if (overrun)
    {
        sensor_errors++;
        fifo_parser_reset(&parser);
        behavior_reset();
        if (sensor_fifo_restart() != RFID_OK)
            sensor_ready = false;
        return;
    }
    if ((!drain && entries < 450) || !entries)
        return;
    /* Timestamp each word against the FIFO occupancy captured before reading.
       ADXL362 ODR is nominal: do not invent samples to force RTC totals. */
    uint32_t start = now - ((uint32_t)entries * 40 / 3);
    status = sensor_fifo_read(fifo_bytes, entries * 2);
    if (status != RFID_OK)
    {
        sensor_ready = false;
        sensor_errors++;
        fifo_parser_reset(&parser);
        behavior_reset();
        return;
    }
#if RFID_DIAGNOSTICS
    char trace[80];
    snprintf(trace, sizeof(trace), "fifo t=%lu words=%u\r\n", (unsigned long)now, entries);
    board_log(trace);
#endif
    for (unsigned i = 0; i < entries; ++i)
    {
        accel_sample_t sample;
        uint16_t word = fifo_bytes[2 * i] | (uint16_t)fifo_bytes[2 * i + 1] << 8;
        uint32_t discarded = parser.discarded;
        bool complete = fifo_parser_word(&parser, word, &sample);
        if (parser.discarded != discarded)
        {
            behavior_reset();
            sensor_errors++;
        }
        if (complete)
        {
            uint8_t result;
            if (behavior_push(sample, &result))
            {
                uint32_t timestamp = start + (i + 1) * 40 / 3;
                /* A classification ending exactly on the boundary belongs to
                   the second just completed, before the window is sealed. */
                advance_to(timestamp - 1);
                history_count(&history, result);
            }
        }
    }
}
void app_init(void)
{
    events = 0;
    report_pending = transmitting = false;
    sensor_errors = radio_errors = storage_errors = 0;
    storage_ready = storage_restore(&store, &history) == RFID_OK;
    if (!storage_ready)
        storage_errors++;
    history.reset_count++;
    if (storage_ready && storage_checkpoint(&store, &history) != RFID_OK)
    {
        storage_ready = false;
        storage_errors++;
    }
    uint64_t seed, stream;
    board_identity(identity, &seed, &stream);
#if RFID_DIAGNOSTICS
    char id_line[48];
    snprintf(id_line, sizeof(id_line), "id=%02X%02X%02X%02X%02X%02X\r\n", identity[0], identity[1],
             identity[2], identity[3], identity[4], identity[5]);
    board_log(id_line);
#endif
    seed ^= (uint64_t)history.reset_count << 16 | board_millis();
    pcg32_srandom_r(&rng, seed, stream);
    uint32_t now = board_rtc_millis();
    period_start = now - (uint32_t)history.elapsed * 1000;
    next_save = period_start + ((uint32_t)history.elapsed / 60 + 1) * 60000;
    behavior_reset();
    fifo_parser_reset(&parser);
    sensor_ready = sensor_init() == RFID_OK;
    if (!sensor_ready)
        sensor_errors++;
    next_sensor_retry = now + 10000;
    log_state("boot");
}
bool app_poll(void)
{
    board_watchdog();
    uint32_t flags = take_events();
    uint32_t now = board_rtc_millis();
    bool checkpoint = (int32_t)(now - next_save) >= 0;
    bool rollover = (int32_t)(now - period_start) >= RFID_PERIOD_SECONDS * 1000;
    if (!sensor_ready && (int32_t)(now - next_sensor_retry) >= 0)
    {
        sensor_ready = sensor_init() == RFID_OK;
        if (!sensor_ready)
            sensor_errors++;
        else
        {
            behavior_reset();
            fifo_parser_reset(&parser);
        }
        next_sensor_retry = now + 10000;
    }
    if (sensor_ready && ((flags & (APP_EVENT_FIFO | APP_EVENT_RTC)) || checkpoint || rollover))
        process_fifo(now, checkpoint || rollover);
    advance_to(now);
    if (checkpoint)
    {
        if (storage_ready && storage_checkpoint(&store, &history) != RFID_OK)
        {
            storage_ready = false;
            storage_errors++;
        }
        next_save = period_start + ((uint32_t)history.elapsed / 60 + 1) * 60000;
        log_state("checkpoint");
    }
    if (report_pending && (int32_t)(board_millis() - next_report) >= 0)
    {
        transmitting = radio_begin(payload, sizeof(payload)) == RFID_OK;
        report_pending = false;
        if (!transmitting)
            radio_errors++;
    }
    if (transmitting)
    {
        rfid_status_t status = radio_poll();
        if (status != RFID_BUSY)
        {
            transmitting = false;
            if (status != RFID_OK)
                radio_errors++;
            log_state(status == RFID_OK ? "tx-ok" : "tx-failed");
        }
    }
    board_watchdog();
    return !report_pending && !transmitting;
}
