#include "history.h"
#include <string.h>
void history_count(rfid_history_t *s, uint8_t behavior)
{
    unsigned index = behavior >= 1 && behavior <= 6 ? behavior - 1 : 5;
    if (s->current[index] != UINT16_MAX)
        ++s->current[index];
}
void history_close(rfid_history_t *s)
{
    for (unsigned c = 0; c < RFID_CLASSES; ++c)
        s->history[c][s->stage] = s->current[c];
    s->stage = (s->stage + 1) % RFID_WINDOWS;
    s->elapsed = 0;
    memset(s->current, 0, sizeof(s->current));
}
