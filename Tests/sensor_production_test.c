#include "board.h"
#include "sensor.h"
#include <assert.h>
#include <stdio.h>
static uint8_t regs[256] = {0xad, 0x1d, 0xf2};
void board_delay(uint32_t ms) { (void)ms; }
void board_pin_write(board_pin_t p, bool level) { (void)p; (void)level; }
void board_spi_enable(unsigned bus, bool on) { (void)on; assert(bus == 2); }
rfid_status_t board_spi_exchange(unsigned bus, uint8_t *data, size_t n)
{
    assert(bus == 2 && n >= 3);
    uint8_t command = data[0], address = data[1];
    for (size_t i = 2; i < n; ++i)
    {
        if (command == 0x0a) regs[address++] = data[i];
        else { assert(command == 0x0b); data[i] = regs[address++]; }
    }
    return RFID_OK;
}
int main(void)
{
    assert(sensor_init() == RFID_OK);
    assert(regs[0x2c] == 0x52 && regs[0x2d] == 2);
    assert(regs[0x28] == 0x0a && regs[0x29] == 0xc2);
    assert(regs[0x27] == 0x3f && regs[0x2b] == 4);
    puts("PASS: production 50Hz/4g/half, unchanged FIFO and activity settings");
    return 0;
}
