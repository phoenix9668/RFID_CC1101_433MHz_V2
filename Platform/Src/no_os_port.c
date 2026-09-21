#include "board.h"
#include "sensor.h"
#include "adxl362.h"
#include "no_os_alloc.h"
#include <string.h>
static struct adxl362_dev device_storage;
static struct no_os_spi_desc spi_storage;
static bool allocated;
static rfid_status_t error;
void *no_os_malloc(size_t size)
{
    if (allocated || size != sizeof(device_storage))
        return NULL;
    allocated = true;
    memset(&device_storage, 0, sizeof(device_storage));
    return &device_storage;
}
void no_os_free(void *ptr)
{
    if (ptr == &device_storage)
        allocated = false;
}
void sensor_port_clear_error(void)
{
    error = RFID_OK;
}
rfid_status_t sensor_port_error(void)
{
    return error;
}
int32_t no_os_spi_init(struct no_os_spi_desc **desc, const struct no_os_spi_init_param *param)
{
    if (!desc || !param || param->mode != 0 || param->max_speed_hz != 4000000)
        return -1;
    spi_storage.active = 1;
    *desc = &spi_storage;
    return 0;
}
int32_t no_os_spi_remove(struct no_os_spi_desc *desc)
{
    if (desc != &spi_storage)
        return -1;
    spi_storage.active = 0;
    return 0;
}
int32_t no_os_spi_write_and_read(struct no_os_spi_desc *desc, uint8_t *data, uint16_t length)
{
    if (error != RFID_OK)
        return -1;
    if (desc != &spi_storage || !desc->active || !data || !length)
    {
        error = RFID_INVALID;
        return -1;
    }
    board_spi_enable(2, true);
    board_pin_write(PIN_SENSOR_CS, false);
    error = board_spi_exchange(2, data, length);
    board_pin_write(PIN_SENSOR_CS, true);
    board_spi_enable(2, false);
    if (error != RFID_OK)
        memset(data, 0, length);
    return error == RFID_OK ? 0 : -1;
}
