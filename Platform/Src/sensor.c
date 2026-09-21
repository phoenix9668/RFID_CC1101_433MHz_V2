#include "sensor.h"
#include "board.h"
#include "adxl362.h"
#include <string.h>
static struct adxl362_dev *device;
static rfid_status_t write_register(uint8_t reg, uint8_t value)
{
    if (!device)
        return RFID_INVALID;
    sensor_port_clear_error();
    adxl362_set_register_value(device, value, reg, 1);
    return sensor_port_error();
}
rfid_status_t sensor_read_register(uint8_t address, uint8_t *value)
{
    if (!device || !value)
        return RFID_INVALID;
    *value = 0;
    sensor_port_clear_error();
    adxl362_get_register_value(device, value, address, 1);
    return sensor_port_error();
}
rfid_status_t sensor_init(void)
{
    if (device)
    {
        adxl362_remove(device);
        device = NULL;
    }
    sensor_port_clear_error();
    struct adxl362_init_param param = {.spi_init = {.max_speed_hz = 4000000, .mode = 0}};
    if (adxl362_init(&device, param) || sensor_port_error() != RFID_OK)
        return RFID_IO;
    adxl362_software_reset(device);
    if (sensor_port_error() != RFID_OK)
        return sensor_port_error();
    board_delay(10);
    const uint8_t ids[3] = {0xad, 0x1d, 0xf2};
    for (unsigned i = 0; i < 3; ++i)
    {
        uint8_t value;
        if (sensor_read_register(i, &value) != RFID_OK || value != ids[i])
            return RFID_IO;
    }
    static const uint8_t config[][2] = {
        {ADXL362_REG_THRESH_ACT_L, 0x64}, {ADXL362_REG_THRESH_ACT_H, 0},
        {ADXL362_REG_TIME_ACT, 6},        {ADXL362_REG_THRESH_INACT_L, 0x64},
        {ADXL362_REG_THRESH_INACT_H, 0},  {ADXL362_REG_TIME_INACT_L, 6},
        {ADXL362_REG_TIME_INACT_H, 0},    {ADXL362_REG_ACT_INACT_CTL, 0x3f},
        {ADXL362_REG_INTMAP1, 0x10},      {ADXL362_REG_INTMAP2, 0x04},
        {ADXL362_REG_FIFO_CTL, 0x0a},     {ADXL362_REG_FIFO_SAMPLES, 0xc2},
        {ADXL362_REG_FILTER_CTL, 0x51},   {ADXL362_REG_POWER_CTL, 0x02}};
    for (unsigned i = 0; i < sizeof(config) / sizeof(config[0]); ++i)
    {
        uint8_t value;
        rfid_status_t status = write_register(config[i][0], config[i][1]);
        if (status != RFID_OK)
            return status;
        status = sensor_read_register(config[i][0], &value);
        if (status != RFID_OK || value != config[i][1])
            return RFID_IO;
    }
    device->selected_range = 4;
#if RFID_DIAGNOSTICS
    board_log("ADXL362 ids=AD/1D/F2 config=ok fifo=450 words 4g/25Hz/half\r\n");
#endif
    return RFID_OK;
}
rfid_status_t sensor_fifo_entries(uint16_t *entries, bool *overrun)
{
    if (!device || !entries || !overrun)
        return RFID_INVALID;
    *entries = 0;
    *overrun = false;
    uint8_t status, bytes[2] = {0};
    rfid_status_t result = sensor_read_register(ADXL362_REG_STATUS, &status);
    if (result != RFID_OK)
        return result;
    sensor_port_clear_error();
    adxl362_get_register_value(device, bytes, ADXL362_REG_FIFO_L, 2);
    if (sensor_port_error() != RFID_OK)
        return sensor_port_error();
    *entries = bytes[0] | (uint16_t)(bytes[1] & 3) << 8;
    *overrun = (status & 8) != 0;
    return *entries <= 512 ? RFID_OK : RFID_IO;
}
rfid_status_t sensor_fifo_read(uint8_t *data, uint16_t length)
{
    if (!device || !data || !length || length > 1024 || (length & 1))
        return RFID_INVALID;
    memset(data, 0, length);
    sensor_port_clear_error();
    for (uint16_t offset = 0; offset < length;)
    {
        uint16_t chunk = length - offset;
        if (chunk > 510)
            chunk = 510;
        adxl362_get_fifo_value(device, data + offset, chunk);
        if (sensor_port_error() != RFID_OK)
        {
            memset(data, 0, length);
            return sensor_port_error();
        }
        offset += chunk;
    }
    return RFID_OK;
}
rfid_status_t sensor_fifo_restart(void)
{
    rfid_status_t status = write_register(ADXL362_REG_FIFO_CTL, 0);
    return status == RFID_OK ? write_register(ADXL362_REG_FIFO_CTL, 0x0a) : status;
}
