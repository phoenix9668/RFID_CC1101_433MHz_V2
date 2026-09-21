/* STM32 port based on TI SWRS061I and SWRZ020E; not a TI-authored driver.
   SWRC021 is the official example reference; no unavailable source is claimed. */
#include "radio.h"
#include "board.h"
#include <string.h>
enum
{
    REG_IOCFG2 = 0,
    REG_IOCFG0 = 2,
    REG_PKTCTRL1 = 7,
    REG_ADDR = 9,
    REG_TXBYTES = 0x3a,
    REG_RXBYTES = 0x3b,
    REG_MARCSTATE = 0x35,
    CMD_SRES = 0x30,
    CMD_SRX = 0x34,
    CMD_STX = 0x35,
    CMD_SIDLE = 0x36,
    CMD_SPWD = 0x39,
    CMD_SFRX = 0x3a,
    CMD_SFTX = 0x3b
};
static const uint8_t config[47] = {
    0x29, 0x2e, 0x46, 0x4e, 0xd3, 0x91, 0xff, 0x07, 0x45, 0x00, 0x00, 0x0b, 0x00, 0x10, 0xa7, 0x62,
    0x7b, 0x83, 0x9b, 0x22, 0xf8, 0x42, 0x03, 0x30, 0x18, 0x1d, 0x1c, 0xc7, 0x00, 0xb2, 0x8c, 0xa0,
    0x78, 0xb6, 0x10, 0xea, 0x2a, 0x00, 0x1f, 0x41, 0x00, 0x59, 0x7f, 0x3f, 0x81, 0x35, 0x09};
static union
{
    uint8_t tx[254];
    uint8_t rx[257];
} packet;
static enum { OFF, POWER_WAIT, TRANSMITTING } phase;
static size_t packet_length, cursor;
static uint32_t deadline, tx_started;
static rfid_status_t last_status;
static bool saw_start;
uint8_t radio_config_byte(unsigned index)
{
    return index < sizeof(config) ? config[index] : 0;
}
static bool expired(uint32_t time)
{
    return (int32_t)(board_millis() - time) >= 0;
}
static rfid_status_t transaction(uint8_t command, uint8_t *data, size_t length)
{
    bool reset = command == CMD_SRES;
    board_pin_write(PIN_RADIO_CS, false);
    uint32_t ready_deadline = board_millis() + 5;
    unsigned spins = 100000;
    while (board_pin_read(PIN_RADIO_SO))
    {
        if (expired(ready_deadline) || !--spins)
        {
            board_pin_write(PIN_RADIO_CS, true);
            return RFID_TIMEOUT;
        }
    }
    rfid_status_t result = board_spi_exchange(1, &command, 1);
    if (result == RFID_OK && length)
        result = board_spi_exchange(1, data, length);
    if (result == RFID_OK && reset)
    {
        /* SRES completes only when SO falls again; keep CS asserted until then. */
        ready_deadline = board_millis() + 5;
        spins = 100000;
        while (board_pin_read(PIN_RADIO_SO))
            if (expired(ready_deadline) || !--spins)
            {
                result = RFID_TIMEOUT;
                break;
            }
    }
    board_pin_write(PIN_RADIO_CS, true);
    return result;
}
static rfid_status_t strobe(uint8_t command)
{
    return transaction(command, NULL, 0);
}
static rfid_status_t write_reg(uint8_t reg, uint8_t value)
{
    return transaction(reg, &value, 1);
}
static rfid_status_t read_reg(uint8_t reg, uint8_t *value)
{
    *value = 0;
    return transaction(reg | 0x80, value, 1);
}
rfid_status_t radio_read_status(uint8_t address, uint8_t *value)
{
    if (!value || address < 0x30 || address > 0x3d)
        return RFID_INVALID;
    *value = 0;
    return transaction(address | 0xc0, value, 1);
}
static rfid_status_t stable_status(uint8_t reg, uint8_t *value)
{
    uint8_t previous;
    rfid_status_t status = radio_read_status(reg, &previous);
    if (status != RFID_OK)
        return status;
    for (unsigned attempt = 0; attempt < 8; ++attempt)
    {
        status = radio_read_status(reg, value);
        if (status != RFID_OK)
            return status;
        if (*value == previous)
            return RFID_OK;
        previous = *value;
    }
    return RFID_IO;
}
static rfid_status_t write_fifo(const uint8_t *data, size_t length)
{
    /* A transaction is at most 60 bytes; exchange may overwrite its buffer. */
    uint8_t buffer[60];
    if (length > sizeof(buffer))
        return RFID_INVALID;
    memcpy(buffer, data, length);
    return transaction(0x7f, buffer, length);
}
static rfid_status_t read_fifo(uint8_t *data, size_t length)
{
    memset(data, 0, length);
    return transaction(0xff, data, length);
}
static rfid_status_t initialize(void)
{
    board_pin_write(PIN_RADIO_CS, true);
    board_delay(1);
    board_pin_write(PIN_RADIO_CS, false);
    board_delay(1);
    board_pin_write(PIN_RADIO_CS, true);
    board_delay(1);
    rfid_status_t status = strobe(CMD_SRES);
    if (status != RFID_OK)
        return status;
    for (unsigned i = 0; i < sizeof(config); ++i)
    {
        status = write_reg(i, config[i]);
        if (status != RFID_OK)
            return status;
    }
    if ((status = write_reg(REG_PKTCTRL1, 0x07)) != RFID_OK ||
        (status = write_reg(REG_ADDR, 0xef)) != RFID_OK ||
        (status = write_reg(4, 0x12)) != RFID_OK || (status = write_reg(5, 0x34)) != RFID_OK)
        return status;
    uint8_t pa[8] = {0xc0};
    status = transaction(0x7e, pa, sizeof(pa));
    if (status != RFID_OK)
        return status;
    uint8_t value;
    status = radio_read_status(0x30, &value);
    if (status != RFID_OK || value != 0)
        return RFID_IO;
    status = radio_read_status(0x31, &value);
    if (status != RFID_OK || value == 0 || value == 0xff)
        return RFID_IO;
    const uint8_t check[][2] = {{7, 7},     {8, 0x45}, {13, 0x10}, {14, 0xa7},
                                {15, 0x62}, {4, 0x12}, {5, 0x34},  {9, 0xef}};
    for (unsigned i = 0; i < sizeof(check) / sizeof(check[0]); ++i)
    {
        status = read_reg(check[i][0], &value);
        if (status != RFID_OK || value != check[i][1])
            return RFID_IO;
    }
    return RFID_OK;
}
static rfid_status_t finish(rfid_status_t result)
{
    /* Cleanup must not replace the original error or wait indefinitely. */
    (void)strobe(CMD_SIDLE);
    (void)strobe(CMD_SFTX);
    (void)strobe(CMD_SFRX);
    (void)strobe(CMD_SPWD);
    board_radio_off();
    phase = OFF;
    last_status = result;
    return result;
}
void radio_abort(void)
{
    if (phase != OFF)
        (void)finish(RFID_IO);
    else
        board_radio_off();
}
rfid_status_t radio_begin(const uint8_t *data, size_t length)
{
    if (phase != OFF)
        return RFID_BUSY;
    if (!data || !length || length > sizeof(packet.tx))
        return RFID_INVALID;
    memcpy(packet.tx, data, length);
    packet_length = length;
    cursor = 0;
    saw_start = false;
    board_pin_write(PIN_TX_EN, false);
    board_pin_write(PIN_RX_EN, false);
    board_pin_write(PIN_RADIO_POWER, true);
    board_spi_enable(1, true);
    phase = POWER_WAIT;
    deadline = board_millis() + 5;
    last_status = RFID_BUSY;
    return RFID_OK;
}
rfid_status_t radio_poll(void)
{
    if (phase == OFF)
        return last_status;
    if (phase == POWER_WAIT)
    {
        if (!expired(deadline))
            return RFID_BUSY;
        rfid_status_t status = initialize();
        if (status != RFID_OK)
            return finish(status);
        if ((status = strobe(CMD_SIDLE)) != RFID_OK || (status = strobe(CMD_SFTX)) != RFID_OK ||
            (status = write_reg(REG_IOCFG2, 0x02)) != RFID_OK)
            return finish(status);
        uint8_t prefix[2] = {(uint8_t)(packet_length + 1), 0xef};
        status = write_fifo(prefix, 2);
        if (status != RFID_OK)
            return finish(status);
        cursor = packet_length < 60 ? packet_length : 60;
        status = write_fifo(packet.tx, cursor);
        if (status != RFID_OK)
            return finish(status);
        board_pin_write(PIN_RX_EN, false);
        board_pin_write(PIN_TX_EN, true);
        status = strobe(CMD_STX);
        if (status != RFID_OK)
            return finish(status);
        tx_started = board_millis();
        deadline = tx_started + 250;
        phase = TRANSMITTING;
    }
    if (expired(deadline))
        return finish(RFID_TIMEOUT);
    uint8_t count, state;
    rfid_status_t status = stable_status(REG_TXBYTES, &count);
    if (status != RFID_OK)
        return finish(status);
    if ((count & 0x80) || count > 64)
        return finish(RFID_OVERFLOW);
    status = stable_status(REG_MARCSTATE, &state);
    if (status != RFID_OK)
        return finish(status);
    if ((state & 0x1f) == 0x16)
        return finish(RFID_OVERFLOW);
    if (!board_pin_read(PIN_GDO0) || (state & 0x1f) == 0x13)
        saw_start = true;
    if (cursor < packet_length)
    {
        size_t chunk = packet_length - cursor;
        if (chunk > 60)
            chunk = 60;
        if (64U - count >= chunk)
        {
            status = write_fifo(packet.tx + cursor, chunk);
            if (status != RFID_OK)
                return finish(status);
            cursor += chunk;
            return RFID_BUSY;
        }
    }
    uint32_t minimum_airtime = (uint32_t)((packet_length + 12) * 16000 / 76767);
    if (cursor == packet_length && count == 0 && (state & 0x1f) == 1 && board_pin_read(PIN_GDO0) &&
        (saw_start || board_millis() - tx_started >= minimum_airtime))
        return finish(RFID_OK);
    return RFID_BUSY;
}
rfid_status_t radio_receive(uint8_t *data, size_t capacity, size_t *length, uint32_t timeout_ms)
{
    if (!data || !length || !capacity || timeout_ms == 0 || timeout_ms > 250)
        return RFID_INVALID;
    *length = 0;
    if (phase != OFF)
        return RFID_BUSY;
    board_pin_write(PIN_RADIO_POWER, true);
    board_spi_enable(1, true);
    board_delay(5);
    phase = POWER_WAIT;
    rfid_status_t status = initialize();
    if (status != RFID_OK)
        return finish(status);
    if ((status = strobe(CMD_SIDLE)) != RFID_OK || (status = strobe(CMD_SFRX)) != RFID_OK ||
        (status = write_reg(REG_IOCFG2, 0x40)) != RFID_OK)
        return finish(status);
    board_pin_write(PIN_TX_EN, false);
    board_pin_write(PIN_RX_EN, true);
    if ((status = strobe(CMD_SRX)) != RFID_OK)
        return finish(status);
    uint32_t end = board_millis() + timeout_ms;
    size_t expected = 0, received = 0;
    while (!expired(end))
    {
        uint8_t count;
        status = stable_status(REG_RXBYTES, &count);
        if (status != RFID_OK)
            return finish(status);
        if ((count & 0x80) || count > 64)
            return finish(RFID_OVERFLOW);
        if (!expected && count >= 2)
        {
            uint8_t n;
            if ((status = read_fifo(&n, 1)) != RFID_OK)
                return finish(status);
            if (n < 1 || (size_t)n - 1 > capacity)
                return finish(RFID_INVALID);
            expected = (size_t)n + 2;
            count--;
        }
        if (expected && count)
        {
            size_t remaining = expected - received;
            /* SWRZ020E: keep one byte until the full remaining packet is available. */
            size_t take = count >= remaining ? remaining : (size_t)count - 1;
            if (take)
            {
                status = read_fifo(packet.rx + received, take);
                if (status != RFID_OK)
                    return finish(status);
                received += take;
            }
            if (received == expected)
            {
                if (!(packet.rx[expected - 1] & 0x80))
                    return finish(RFID_CRC);
                if (packet.rx[0] != 0xef && packet.rx[0] != 0 && packet.rx[0] != 0xff)
                    return finish(RFID_INVALID);
                *length = expected - 3;
                memcpy(data, packet.rx + 1, *length);
                return finish(RFID_OK);
            }
        }
        board_delay(1);
    }
    return finish(RFID_TIMEOUT);
}
