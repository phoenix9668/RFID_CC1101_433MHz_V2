#ifndef RFID_NO_OS_SPI_H
#define RFID_NO_OS_SPI_H
#include <stdint.h>
struct no_os_spi_desc
{
    uint8_t active;
};
struct no_os_spi_init_param
{
    uint32_t max_speed_hz;
    uint8_t mode;
};
int32_t no_os_spi_init(struct no_os_spi_desc **desc, const struct no_os_spi_init_param *param);
int32_t no_os_spi_remove(struct no_os_spi_desc *desc);
int32_t no_os_spi_write_and_read(struct no_os_spi_desc *desc, uint8_t *data, uint16_t length);
#endif
