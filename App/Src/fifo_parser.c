#include "fifo_parser.h"
#include <string.h>
void fifo_parser_reset(fifo_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}
bool fifo_parser_word(fifo_parser_t *p, uint16_t word, accel_sample_t *out)
{
    unsigned axis = word >> 14;
    int16_t value = (int16_t)(word & 0x0fff);
    if (value & 0x0800)
        value = (int16_t)(value - 4096);
    if (axis == 0)
    {
        if (p->next_axis != 0)
            p->discarded++;
        p->partial.x = value;
        p->next_axis = 1;
    }
    else if (axis == 1 && p->next_axis == 1)
    {
        p->partial.y = value;
        p->next_axis = 2;
    }
    else if (axis == 2 && p->next_axis == 2)
    {
        p->partial.z = value;
        *out = p->partial;
        p->next_axis = 0;
        return true;
    }
    else
    {
        p->next_axis = 0;
        p->discarded++;
    }
    return false;
}
