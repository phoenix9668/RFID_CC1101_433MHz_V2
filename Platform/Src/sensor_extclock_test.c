/* Isolated A/B experiment: no application, radio, storage or STOP calls. */
#include "sensor_extclock_test.h"
#include "sensor_test_clock.h"
#include "sensor.h"
#include "board.h"
#include "adxl362.h"
#include <stdio.h>

static rfid_status_t measure(unsigned phase, bool external)
{
    char line[192];
    sensor_test_clock_stop();
    rfid_status_t status = sensor_init();
    if (status == RFID_OK) status = sensor_clock_test_prepare();
    if (status == RFID_OK && external) status = sensor_test_clock_start();
    if (status == RFID_OK) status = sensor_clock_test_measure(external);
    uint8_t filter = 0, power = 0, revision = 0;
    if (status == RFID_OK) status = sensor_read_register(ADXL362_REG_FILTER_CTL, &filter);
    if (status == RFID_OK) status = sensor_read_register(ADXL362_REG_POWER_CTL, &power);
    if (status == RFID_OK) status = sensor_read_register(ADXL362_REG_REVID, &revision);
    snprintf(line, sizeof(line),
             "EXTCLK phase=%u source=%s ref_hz=%u fifo=00 int1=00 int2=01 filter=%02X power=%02X rev=%02X status=%u\r\n",
             phase, external ? "PB0" : "INTERNAL", external ? 32000 : 0,
             filter, power, revision, (unsigned)status);
    board_log(line);
    /* More than 4/ODR at either expected rate; clear startup data after settling. */
    for (unsigned i = 0; status == RFID_OK && i < 6; ++i)
    {
        board_watchdog();
        board_delay(100);
        if (external) status = sensor_test_clock_status();
    }
    uint8_t discard;
    bool armed = false;
    /* A new sample may arrive during the clearing read. Establish a low level
     * before measuring, instead of treating an unobserved low as a stuck pin. */
    for (unsigned attempt = 0; status == RFID_OK && !armed && attempt < 3; ++attempt)
    {
        status = sensor_read_register(ADXL362_REG_XDATA_L, &discard);
        board_delay(1);
        if (status == RFID_OK) armed = !board_pin_read(PIN_SENSOR_IRQ);
    }
    if (status == RFID_OK && !armed) status = RFID_TIMEOUT;
    uint32_t start = board_millis(), last = start, first = 0, count = 0;
    uint32_t first_rtc = 0, last_rtc = 0, minimum = UINT32_MAX, maximum = 0;
    while (status == RFID_OK && (uint32_t)(board_millis() - start) < 10000)
    {
        board_watchdog();
        if (external && (status = sensor_test_clock_status()) != RFID_OK) break;
        uint32_t now = board_millis();
        bool high = board_pin_read(PIN_SENSOR_IRQ);
        if (!high) armed = true;
        else if (armed)
        {
            uint32_t rtc = board_rtc_millis();
            if (count)
            {
                uint32_t delta = now - last;
                if (delta < minimum) minimum = delta;
                if (delta > maximum) maximum = delta;
            }
            else { first = now; first_rtc = rtc; }
            last = now; last_rtc = rtc; ++count; armed = false;
            status = sensor_read_register(ADXL362_REG_XDATA_L, &discard);
        }
        else if ((uint32_t)(now - last) > 5) status = RFID_TIMEOUT;
        if ((uint32_t)(now - last) > 250) status = RFID_TIMEOUT;
        board_delay(1);
    }
    if (status != RFID_OK)
    {
        uint8_t sensor_status = 0, map = 0;
        bool pin = board_pin_read(PIN_SENSOR_IRQ);
        rfid_status_t a = sensor_read_register(ADXL362_REG_STATUS, &sensor_status);
        rfid_status_t b = sensor_read_register(ADXL362_REG_INTMAP2, &map);
        snprintf(line, sizeof(line), "EXTCLK FAULT phase=%u armed=%u pin=%u sensor_status=%02X int2=%02X io=%u/%u\r\n",
                 phase, armed, pin, sensor_status, map, (unsigned)a, (unsigned)b);
        board_log(line);
    }
    rfid_status_t stopped = sensor_clock_test_end();
    if (status == RFID_OK) status = stopped;
    if (external && status == RFID_OK) status = sensor_test_clock_status();
    sensor_test_clock_stop();
    sensor_test_clock_stats_t clock = external ? sensor_test_clock_stats() : (sensor_test_clock_stats_t){0};
    snprintf(line, sizeof(line),
             "EXTCLK END phase=%u n=%lu tick_span=%lu rtc_span=%lu dt_min=%lu dt_max=%lu status=%u stop=%u\r\n",
             phase, (unsigned long)count, (unsigned long)(count ? last-first : 0),
             (unsigned long)(count ? last_rtc-first_rtc : 0),
             (unsigned long)(count > 1 ? minimum : 0), (unsigned long)maximum,
             (unsigned)status, (unsigned)stopped);
    board_log(line);
    snprintf(line, sizeof(line), "EXTCLK REF phase=%u edges=%lu max_latency_ticks=%lu late=%u\r\n",
             phase, (unsigned long)clock.edges, (unsigned long)clock.max_latency_ticks, clock.late);
    board_log(line);
    return status;
}

rfid_status_t sensor_extclock_test_run(void)
{
    board_log("EXTCLK BEGIN TEST_ONLY: 24 phases INTERNAL/PB0; no classification/RF/EEPROM/STOP\r\n");
    for (unsigned phase = 0; phase < 24; ++phase)
    {
        rfid_status_t status = measure(phase, (phase & 1) != 0);
        if (status != RFID_OK) return status;
        for (unsigned i = 0; i < 20; ++i) { board_watchdog(); board_delay(100); }
    }
    board_log("EXTCLK COMPLETE: sensor standby, PB0 high-Z; restore normal image\r\n");
    return RFID_OK;
}
