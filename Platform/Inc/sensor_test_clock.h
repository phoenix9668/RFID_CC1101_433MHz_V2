#ifndef RFID_SENSOR_TEST_CLOCK_H
#define RFID_SENSOR_TEST_CLOCK_H
#include "rfid_types.h"
typedef struct {
    uint32_t edges;
    uint32_t max_latency_ticks;
    bool late;
} sensor_test_clock_stats_t;
rfid_status_t sensor_test_clock_start(void);
rfid_status_t sensor_test_clock_status(void);
sensor_test_clock_stats_t sensor_test_clock_stats(void);
void sensor_test_clock_stop(void);
#endif
