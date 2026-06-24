#include <stdint.h>
#include <string.h>

#include "gpio.h"
#include "no_os_spi.h"
#include "spi.h"

#define NO_OS_SPI_TIMEOUT 100000U

static struct no_os_spi_desc spi_desc_storage;

static int32_t no_os_spi_exchange_byte(SPI_TypeDef *SPIx, uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = NO_OS_SPI_TIMEOUT;

    while (!LL_SPI_IsActiveFlag_TXE(SPIx))
    {
        if (timeout-- == 0U)
            return -1;
    }

    LL_SPI_TransmitData8(SPIx, tx);

    timeout = NO_OS_SPI_TIMEOUT;
    while (!LL_SPI_IsActiveFlag_RXNE(SPIx))
    {
        if (timeout-- == 0U)
            return -1;
    }

    *rx = LL_SPI_ReceiveData8(SPIx);
    return 0;
}

int32_t no_os_spi_init(struct no_os_spi_desc **desc,
                       const struct no_os_spi_init_param *param)
{
    if (desc == NULL || param == NULL)
        return -1;

    memset(&spi_desc_storage, 0, sizeof(spi_desc_storage));
    spi_desc_storage.device_id = param->device_id;
    spi_desc_storage.max_speed_hz = param->max_speed_hz;
    spi_desc_storage.chip_select = param->chip_select;
    spi_desc_storage.mode = param->mode;
    spi_desc_storage.bit_order = param->bit_order;
    spi_desc_storage.lanes = param->lanes;
    spi_desc_storage.platform_ops = param->platform_ops;
    spi_desc_storage.platform_delays = param->platform_delays;
    spi_desc_storage.extra = param->extra;
    spi_desc_storage.parent = param->parent;

    *desc = &spi_desc_storage;
    return 0;
}

int32_t no_os_spi_remove(struct no_os_spi_desc *desc)
{
    (void)desc;
    return 0;
}

int32_t no_os_spi_write_and_read(struct no_os_spi_desc *desc,
                                 uint8_t *data,
                                 uint16_t bytes_number)
{
    SPI_TypeDef *SPIx;

    if (desc == NULL || data == NULL)
        return -1;

    SPIx = (SPI_TypeDef *)desc->extra;
    if (SPIx == NULL)
        SPIx = SPI2;

    ADXL362_CSN_LOW();
    for (uint16_t index = 0; index < bytes_number; index++)
    {
        uint8_t rx = 0;
        if (no_os_spi_exchange_byte(SPIx, data[index], &rx) != 0)
        {
            ADXL362_CSN_HIGH();
            return -1;
        }
        data[index] = rx;
    }
    ADXL362_CSN_HIGH();

    return 0;
}

int32_t no_os_spi_transfer(struct no_os_spi_desc *desc,
                           struct no_os_spi_msg *msgs,
                           uint32_t len)
{
    SPI_TypeDef *SPIx;

    if (desc == NULL || msgs == NULL)
        return -1;

    SPIx = (SPI_TypeDef *)desc->extra;
    if (SPIx == NULL)
        SPIx = SPI2;

    ADXL362_CSN_LOW();
    for (uint32_t msg_index = 0; msg_index < len; msg_index++)
    {
        for (uint32_t byte_index = 0; byte_index < msgs[msg_index].bytes_number; byte_index++)
        {
            uint8_t tx = msgs[msg_index].tx_buff ? msgs[msg_index].tx_buff[byte_index] : 0xFFU;
            uint8_t rx = 0;

            if (no_os_spi_exchange_byte(SPIx, tx, &rx) != 0)
            {
                ADXL362_CSN_HIGH();
                return -1;
            }

            if (msgs[msg_index].rx_buff)
                msgs[msg_index].rx_buff[byte_index] = rx;
        }

        if (msgs[msg_index].cs_change)
        {
            ADXL362_CSN_HIGH();
            if (msg_index + 1U < len)
                ADXL362_CSN_LOW();
        }
    }
    ADXL362_CSN_HIGH();

    return 0;
}

int32_t no_os_spi_transfer_dma(struct no_os_spi_desc *desc,
                               struct no_os_spi_msg *msgs,
                               uint32_t len)
{
    return no_os_spi_transfer(desc, msgs, len);
}

int32_t no_os_spi_transfer_dma_async(struct no_os_spi_desc *desc,
                                     struct no_os_spi_msg *msgs,
                                     uint32_t len,
                                     void (*callback)(void *),
                                     void *ctx)
{
    int32_t ret = no_os_spi_transfer(desc, msgs, len);

    if (callback != NULL)
        callback(ctx);

    return ret;
}

int32_t no_os_spi_transfer_abort(struct no_os_spi_desc *desc)
{
    (void)desc;
    ADXL362_CSN_HIGH();
    return 0;
}

int32_t no_os_spibus_init(const struct no_os_spi_init_param *param)
{
    (void)param;
    return 0;
}

void no_os_spibus_remove(uint32_t bus_number)
{
    (void)bus_number;
}
