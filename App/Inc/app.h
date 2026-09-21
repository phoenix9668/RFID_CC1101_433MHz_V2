#ifndef RFID_APP_H
#define RFID_APP_H
#include <stdbool.h>
#include <stdint.h>
#include "rfid_types.h"
enum
{
    APP_EVENT_FIFO = 1,
    APP_EVENT_RTC = 2,
    APP_EVENT_RADIO = 4
};
void app_init(void);
bool app_poll(void);
void app_signal(uint32_t events);
bool app_events_pending(void);
/* Main-context RTC delay, capped at 10 seconds; zero means work is due. */
uint32_t app_wakeup_delay_ms(void);
/* Main-context diagnostic snapshot; never exposes mutable application state. */
void app_snapshot(rfid_history_t *out);
#endif
