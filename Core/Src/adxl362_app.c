#include <stdint.h>
#include <string.h>

#include "adxl362.h"
#include "main.h"
#include "spi.h"
#include "usart.h"

#ifndef ADXL362_PC_RUNNER
#include "../../Drivers/ADI/no-OS/drivers/accel/adxl362/adxl362.h"

#define ADXL362_CONFIG_ERROR_INDEX 0x03U
#define ADXL362_FIFO_READ_CHUNK    511U
#define ADXL362_REG_READ_CHUNK     30U
#define ADXL362_SPI_DEVICE_ID      2U
#define ADXL362_SPI_MAX_SPEED_HZ   8000000U
#define ADXL362_FIFO_WATERMARK     0x01C2U

extern uint8_t ErrorIndex;

static struct adxl362_dev *adxl362_device;

static void ADXL362ConfigError(void)
{
    ErrorIndex = ADXL362_CONFIG_ERROR_INDEX;
    Error_Handler();
}

static int32_t ADXL362EnsureDevice(void)
{
    struct adxl362_dev *device = NULL;
    int32_t ret;

    if (adxl362_device != NULL)
        return 0;

    struct adxl362_init_param init_param;
    memset(&init_param, 0, sizeof(init_param));
    init_param.spi_init.device_id = ADXL362_SPI_DEVICE_ID;
    init_param.spi_init.max_speed_hz = ADXL362_SPI_MAX_SPEED_HZ;
    init_param.spi_init.chip_select = 0;
    init_param.spi_init.mode = NO_OS_SPI_MODE_0;
    init_param.spi_init.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST;
    init_param.spi_init.lanes = NO_OS_SPI_SINGLE_LANE;
    init_param.spi_init.extra = SPI2;

    ret = adxl362_init(&device, init_param);
    if (ret != 0)
    {
        if (device != NULL)
            (void)adxl362_remove(device);
        return ret;
    }

    adxl362_device = device;
    return 0;
}

static uint8_t ADXL362ReadAndCheck(uint8_t reg, uint8_t expected, const char *label)
{
    uint8_t value = ADXL362RegisterRead(reg);

    rfid_printf("|*-set %s register = 0x%02x-*|\n", label, value);
    if (value != expected)
        ADXL362ConfigError();

    return value;
}

unsigned char ADXL362RegisterRead(unsigned char Address)
{
    uint8_t value = 0;

    if (ADXL362EnsureDevice() != 0)
        return 0;

    adxl362_get_register_value(adxl362_device, &value, Address, 1);
    return value;
}

void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue)
{
    if (ADXL362EnsureDevice() != 0)
        return;

    adxl362_set_register_value(adxl362_device, SendValue, Address, 1);
}

void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
{
    uint8_t offset = 0;

    if (RegisterData == NULL || ADXL362EnsureDevice() != 0)
        return;

    while (offset < NumberofRegisters)
    {
        uint8_t chunk = NumberofRegisters - offset;
        if (chunk > ADXL362_REG_READ_CHUNK)
            chunk = ADXL362_REG_READ_CHUNK;

        adxl362_get_register_value(adxl362_device,
                                   &RegisterData[offset],
                                   (uint8_t)(Address + offset),
                                   chunk);
        offset = (uint8_t)(offset + chunk);
    }
}

void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
{
    if (RegisterData == NULL || ADXL362EnsureDevice() != 0)
        return;

    for (uint8_t index = 0; index < NumberofRegisters; index++)
    {
        adxl362_set_register_value(adxl362_device,
                                   RegisterData[index],
                                   (uint8_t)(Address + index),
                                   1);
    }
}

void ADXL362FifoRead(unsigned int NumberofRegisters, unsigned char *RegisterData)
{
    uint16_t offset = 0;

    if (RegisterData == NULL || ADXL362EnsureDevice() != 0)
        return;

    while (offset < NumberofRegisters)
    {
        uint16_t chunk = (uint16_t)(NumberofRegisters - offset);
        if (chunk > ADXL362_FIFO_READ_CHUNK)
            chunk = ADXL362_FIFO_READ_CHUNK;

        adxl362_get_fifo_value(adxl362_device, &RegisterData[offset], chunk);
        offset = (uint16_t)(offset + chunk);
    }
}

uint16_t ADXL362FifoEntries(void)
{
    uint8_t low = ADXL362RegisterRead(ADXL362_REG_FIFO_L);
    uint8_t high = ADXL362RegisterRead(ADXL362_REG_FIFO_H);

    return (uint16_t)(low | ((uint16_t)(high & 0x03U) << 8));
}

void ADXL362_Init(void)
{
    memset(&step, 0, sizeof(step));
    memset(three_axis_info, 0, sizeof(three_axis_info));
    memset(memory_array, 0, sizeof(memory_array));
    memset(fifo, 0, sizeof(fifo));

    if (ADXL362EnsureDevice() != 0)
        ADXL362ConfigError();

    adxl362_software_reset(adxl362_device);
    HAL_Delay(1000);

    rfid_printf("\n|********ADXL362 CONFIGURE INFO********|\n");

    adxl362_setup_activity_detection(adxl362_device, 1, 0x0064, 0x06);
    ADXL362ReadAndCheck(ADXL362_REG_THRESH_ACT_L, 0x64, "THRESH_ACT_L");
    ADXL362ReadAndCheck(ADXL362_REG_THRESH_ACT_H, 0x00, "THRESH_ACT_H");
    ADXL362ReadAndCheck(ADXL362_REG_TIME_ACT, 0x06, "TIME_ACT");

    adxl362_setup_inactivity_detection(adxl362_device, 1, 0x0064, 0x0006);
    ADXL362ReadAndCheck(ADXL362_REG_THRESH_INACT_L, 0x64, "THRESH_INACT_L");
    ADXL362ReadAndCheck(ADXL362_REG_THRESH_INACT_H, 0x00, "THRESH_INACT_H");
    ADXL362ReadAndCheck(ADXL362_REG_TIME_INACT_L, 0x06, "TIME_INACT_L");
    ADXL362ReadAndCheck(ADXL362_REG_TIME_INACT_H, 0x00, "TIME_INACT_H");

    ADXL362RegisterWrite(ADXL362_REG_ACT_INACT_CTL,
                         ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LOOP) |
                         ADXL362_ACT_INACT_CTL_INACT_REF |
                         ADXL362_ACT_INACT_CTL_INACT_EN |
                         ADXL362_ACT_INACT_CTL_ACT_REF |
                         ADXL362_ACT_INACT_CTL_ACT_EN);
    ADXL362ReadAndCheck(ADXL362_REG_ACT_INACT_CTL, 0x3F, "ACT_INACT_CTL");

    ADXL362RegisterWrite(ADXL362_REG_INTMAP1, ADXL362_INTMAP1_ACT);
    ADXL362ReadAndCheck(ADXL362_REG_INTMAP1, 0x10, "INTMAP1");

    ADXL362RegisterWrite(ADXL362_REG_INTMAP2, ADXL362_INTMAP2_FIFO_WATERMARK);
    ADXL362ReadAndCheck(ADXL362_REG_INTMAP2, 0x04, "INTMAP2");

    adxl362_fifo_setup(adxl362_device, ADXL362_FIFO_STREAM, ADXL362_FIFO_WATERMARK, 0);
    ADXL362ReadAndCheck(ADXL362_REG_FIFO_CTL, 0x0A, "FIFO_CONTROL");
    ADXL362ReadAndCheck(ADXL362_REG_FIFO_SAMPLES, 0xC2, "FIFO_SAMPLES");

    adxl362_set_range(adxl362_device, ADXL362_RANGE_4G);
    adxl362_set_output_rate(adxl362_device, ADXL362_ODR_25_HZ);
    ADXL362RegisterWrite(ADXL362_REG_FILTER_CTL,
                         ADXL362_FILTER_CTL_RANGE(ADXL362_RANGE_4G) |
                         ADXL362_FILTER_CTL_HALF_BW |
                         ADXL362_FILTER_CTL_ODR(ADXL362_ODR_25_HZ));
    ADXL362ReadAndCheck(ADXL362_REG_FILTER_CTL, 0x51, "FILTER_CTL");

    adxl362_set_power_mode(adxl362_device, 1);
    ADXL362ReadAndCheck(ADXL362_REG_POWER_CTL, 0x02, "POWER_CTL");

    rfid_printf("|**************************************|\n");
    HAL_Delay(200);
}
#endif
