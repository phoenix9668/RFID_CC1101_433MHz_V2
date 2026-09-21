/* Non-production timing experiment. Never call app_init or storage functions. */
#include "sensor_odr_test.h"
#include "sensor.h"
#include "board.h"
#include "adxl362.h"
#include <stdio.h>

rfid_status_t sensor_odr_test_run(uint32_t duration_ms)
{
    if (duration_ms < 1000 || duration_ms > 240000)
        return RFID_INVALID;
    board_log("TEST_ONLY DATA_READY: no classification/RF/EEPROM; STOP disabled\r\n");
    rfid_status_t status = sensor_init();
    if (status == RFID_OK)
        status = sensor_data_ready_begin();
    uint8_t revision = 0;
    if (status == RFID_OK)
        status = sensor_read_register(ADXL362_REG_REVID, &revision);
    char line[160];
    snprintf(line, sizeof(line), "DRDY config fifo=00 int2=01 filter=51 power=02 rev=%02X status=%u\r\n",
             revision, (unsigned)status);
    board_log(line);
    uint32_t start = board_millis(), next_log = start + 10000;
    uint32_t count = 0, first = 0, last = start, first_rtc = 0, last_rtc = 0;
    uint32_t minimum = UINT32_MAX, maximum = 0;
    bool armed = false;
    while (status == RFID_OK && (uint32_t)(board_millis() - start) < duration_ms)
    {
        board_watchdog();
        uint32_t now = board_millis();
        bool high = board_pin_read(PIN_SENSOR_IRQ);
        if (!high)
            armed = true;
        else if (armed)
        {
            uint32_t rtc = board_rtc_millis();
            if (count)
            {
                uint32_t delta = now - last;
                if (delta < minimum) minimum = delta;
                if (delta > maximum) maximum = delta;
            }
            else
            {
                first = now;
                first_rtc = rtc;
            }
            last = now;
            last_rtc = rtc;
            ++count;
            armed = false;
            /* Reading data clears DATA_READY; allow its documented 80 us latency. */
            uint8_t discard;
            status = sensor_read_register(ADXL362_REG_XDATA_L, &discard);
        }
        else if ((uint32_t)(now - last) > 5)
            status = RFID_TIMEOUT;
        if ((uint32_t)(now - last) > 250)
            status = RFID_TIMEOUT;
        if ((int32_t)(now - next_log) >= 0)
        {
            snprintf(line, sizeof(line), "drdy n=%lu tick_span=%lu rtc_span=%lu dt_min=%lu dt_max=%lu status=%u\r\n",
                     (unsigned long)count, (unsigned long)(count ? last-first : 0),
                     (unsigned long)(count ? last_rtc-first_rtc : 0),
                     (unsigned long)(count > 1 ? minimum : 0), (unsigned long)maximum, (unsigned)status);
            board_log(line);
            next_log = now + 10000;
        }
        board_delay(1);
    }
    rfid_status_t stopped = sensor_data_ready_end();
    if (status == RFID_OK) status = stopped;
    snprintf(line, sizeof(line), "DRDY END status=%u n=%lu; sensor stop=%u; restore normal image\r\n",
             (unsigned)status, (unsigned long)count, (unsigned)stopped);
    board_log(line);
    return status;
}
