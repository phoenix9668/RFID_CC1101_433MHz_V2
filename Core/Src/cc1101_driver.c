/**
 ******************************************************************************
 * @file    cc1101_driver.c
 * @brief   CC1101 register, FIFO and packet driver for STM32L0.
 *
 * This driver follows the transaction rules used by TI SWRC021 examples:
 * assert CSn, wait until SO/MISO goes low, then send the SPI header byte.
 ******************************************************************************
 */

#include "cc1101.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"

#define CC1101_CONFIG_COUNT       47U
#define CC1101_TX_FIFO_CHUNK      60U
#define CC1101_SPI_TIMEOUT        100000U
#define CC1101_SO_TIMEOUT         100000U
#define CC1101_GDO_TIMEOUT        3000000U

// 10, 7, 5, 0, -5, -10, -15, -20, dbm output power
static uint8_t CC1101PaTable[] = {0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
__IO ITStatus txFiFoUnFlow = RESET;
__IO ITStatus rxCatch = RESET;

static cc1101_status_t lastError = CC1101_STATUS_OK;

// Generated from SmartRF Studio for the existing 433 MHz configuration.
static const uint8_t CC1101InitData[CC1101_CONFIG_COUNT][2] =
{
    {CC1101_IOCFG2, 0x29},
    {CC1101_IOCFG1, 0x2E},
    {CC1101_IOCFG0, 0x46},
    {CC1101_FIFOTHR, 0x4E},
    {CC1101_SYNC1, 0xD3},
    {CC1101_SYNC0, 0x91},
    {CC1101_PKTLEN, 0xFF},
    {CC1101_PKTCTRL1, 0x07},
    {CC1101_PKTCTRL0, 0x45},
    {CC1101_ADDR, 0x00},
    {CC1101_CHANNR, 0x00},
    {CC1101_FSCTRL1, 0x0B},
    {CC1101_FSCTRL0, 0x00},
    {CC1101_FREQ2, 0x10},
    {CC1101_FREQ1, 0xA7},
    {CC1101_FREQ0, 0x62},
    {CC1101_MDMCFG4, 0x7B},
    {CC1101_MDMCFG3, 0x83},
    {CC1101_MDMCFG2, 0x9B},
    {CC1101_MDMCFG1, 0x22},
    {CC1101_MDMCFG0, 0xF8},
    {CC1101_DEVIATN, 0x42},
    {CC1101_MCSM2, 0x03},
    {CC1101_MCSM1, 0x30},
    {CC1101_MCSM0, 0x18},
    {CC1101_FOCCFG, 0x1D},
    {CC1101_BSCFG, 0x1C},
    {CC1101_AGCCTRL2, 0xC7},
    {CC1101_AGCCTRL1, 0x00},
    {CC1101_AGCCTRL0, 0xB2},
    {CC1101_WOREVT1, 0x8C},
    {CC1101_WOREVT0, 0xA0},
    {CC1101_WORCTRL, 0x78},
    {CC1101_FREND1, 0xB6},
    {CC1101_FREND0, 0x10},
    {CC1101_FSCAL3, 0xEA},
    {CC1101_FSCAL2, 0x2A},
    {CC1101_FSCAL1, 0x00},
    {CC1101_FSCAL0, 0x1F},
    {CC1101_RCCTRL1, 0x41},
    {CC1101_RCCTRL0, 0x00},
    {CC1101_FSTEST, 0x59},
    {CC1101_PTEST, 0x7F},
    {CC1101_AGCTEST, 0x3F},
    {CC1101_TEST2, 0x81},
    {CC1101_TEST1, 0x35},
    {CC1101_TEST0, 0x09}
};

static void CC1101SetError(cc1101_status_t error)
{
    if (error != CC1101_STATUS_OK && lastError == CC1101_STATUS_OK)
        lastError = error;
}

static cc1101_status_t CC1101WaitMisoLow(void)
{
    uint32_t timeout = CC1101_SO_TIMEOUT;

    while (CC1101_MISO_READ())
    {
        if (timeout-- == 0U)
        {
            CC1101SetError(CC1101_STATUS_SO_TIMEOUT);
            return CC1101_STATUS_SO_TIMEOUT;
        }
    }

    return CC1101_STATUS_OK;
}

static cc1101_status_t CC1101Select(void)
{
    CC1101_CSN_LOW();
    return CC1101WaitMisoLow();
}

static void CC1101Deselect(void)
{
    CC1101_CSN_HIGH();
}

static cc1101_status_t CC1101ExchangeByte(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = CC1101_SPI_TIMEOUT;

    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
    {
        if (timeout-- == 0U)
        {
            CC1101SetError(CC1101_STATUS_SPI_TIMEOUT);
            return CC1101_STATUS_SPI_TIMEOUT;
        }
    }

    LL_SPI_TransmitData8(SPI1, tx);

    timeout = CC1101_SPI_TIMEOUT;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
        if (timeout-- == 0U)
        {
            CC1101SetError(CC1101_STATUS_SPI_TIMEOUT);
            return CC1101_STATUS_SPI_TIMEOUT;
        }
    }

    if (rx != NULL)
        *rx = LL_SPI_ReceiveData8(SPI1);
    else
        (void)LL_SPI_ReceiveData8(SPI1);

    return CC1101_STATUS_OK;
}

static uint8_t CC1101WaitItFlag(__IO ITStatus *flag,
                                ITStatus expected,
                                cc1101_status_t timeoutError)
{
    uint32_t timeout = CC1101_GDO_TIMEOUT;

    while (*flag != expected)
    {
        if (timeout-- == 0U)
        {
            CC1101SetError(timeoutError);
            return 0;
        }
    }

    return 1;
}

cc1101_status_t CC1101GetLastError(void)
{
    return lastError;
}

void CC1101ClearLastError(void)
{
    lastError = CC1101_STATUS_OK;
}

uint8_t CC1101ReadPartNumber(void)
{
    return CC1101ReadStatus(CC1101_PARTNUM);
}

uint8_t CC1101ReadVersion(void)
{
    return CC1101ReadStatus(CC1101_VERSION);
}

uint8_t CC1101ReadReg(uint8_t addr)
{
    uint8_t value = 0;

    if (CC1101Select() != CC1101_STATUS_OK)
    {
        CC1101Deselect();
        return 0;
    }

    if (CC1101ExchangeByte((uint8_t)(addr | READ_SINGLE), NULL) == CC1101_STATUS_OK)
        (void)CC1101ExchangeByte(0xFF, &value);

    CC1101Deselect();
    return value;
}

void CC1101ReadMultiReg(uint8_t addr, uint8_t *buff, uint8_t size)
{
    if (buff == NULL)
    {
        CC1101SetError(CC1101_STATUS_INVALID_ARGUMENT);
        return;
    }

    if (CC1101Select() != CC1101_STATUS_OK)
    {
        CC1101Deselect();
        return;
    }

    if (CC1101ExchangeByte((uint8_t)(addr | READ_BURST), NULL) == CC1101_STATUS_OK)
    {
        for (uint8_t i = 0; i < size; i++)
        {
            if (CC1101ExchangeByte(0xFF, &buff[i]) != CC1101_STATUS_OK)
                break;
        }
    }

    CC1101Deselect();
}

uint8_t CC1101ReadStatus(uint8_t addr)
{
    uint8_t value = 0;

    if (CC1101Select() != CC1101_STATUS_OK)
    {
        CC1101Deselect();
        return 0;
    }

    if (CC1101ExchangeByte((uint8_t)(addr | READ_BURST), NULL) == CC1101_STATUS_OK)
        (void)CC1101ExchangeByte(0xFF, &value);

    CC1101Deselect();
    return value;
}

void CC1101WriteReg(uint8_t addr, uint8_t value)
{
    if (CC1101Select() != CC1101_STATUS_OK)
    {
        CC1101Deselect();
        return;
    }

    if (CC1101ExchangeByte(addr, NULL) == CC1101_STATUS_OK)
        (void)CC1101ExchangeByte(value, NULL);

    CC1101Deselect();
}

void CC1101WriteMultiReg(uint8_t addr, uint8_t *buff, uint8_t size)
{
    if (buff == NULL)
    {
        CC1101SetError(CC1101_STATUS_INVALID_ARGUMENT);
        return;
    }

    if (CC1101Select() != CC1101_STATUS_OK)
    {
        CC1101Deselect();
        return;
    }

    if (CC1101ExchangeByte((uint8_t)(addr | WRITE_BURST), NULL) == CC1101_STATUS_OK)
    {
        for (uint8_t i = 0; i < size; i++)
        {
            if (CC1101ExchangeByte(buff[i], NULL) != CC1101_STATUS_OK)
                break;
        }
    }

    CC1101Deselect();
}

void CC1101WriteCmd(uint8_t command)
{
    if (CC1101Select() != CC1101_STATUS_OK)
    {
        CC1101Deselect();
        return;
    }

    (void)CC1101ExchangeByte(command, NULL);
    CC1101Deselect();
}

void CC1101WORInit(void)
{
    CC1101WriteReg(CC1101_MCSM0, 0x18);
    CC1101WriteReg(CC1101_WORCTRL, 0x78);
    CC1101WriteReg(CC1101_MCSM2, 0x03);
    CC1101WriteReg(CC1101_WOREVT1, 0x8C);
    CC1101WriteReg(CC1101_WOREVT0, 0xA0);
}

void CC1101SetWORMode(void)
{
    CC1101WriteReg(CC1101_IOCFG0, 0x46);
    CC1101WriteReg(CC1101_IOCFG2, 0x00);
    CC1101WriteCmd(CC1101_SFRX);
    CC1101WriteCmd(CC1101_SWORRST);
    CC1101WriteCmd(CC1101_SWOR);
}

void CC1101SetTRMode(TRMODE mode)
{
    if (mode == TX_MODE)
    {
        RX_EN_LOW();
        TX_EN_HIGH();
        CC1101WriteReg(CC1101_IOCFG0, 0x46);
        CC1101WriteReg(CC1101_IOCFG2, 0x02);
        CC1101WriteCmd(CC1101_STX);
    }
    else if (mode == RX_MODE)
    {
        TX_EN_LOW();
        RX_EN_HIGH();
        CC1101WriteReg(CC1101_IOCFG0, 0x46);
        CC1101WriteReg(CC1101_IOCFG2, 0x40);
        CC1101WriteCmd(CC1101_SRX);
    }
}

void CC1101Reset(void)
{
    CC1101_CSN_HIGH();
    HAL_Delay(1);
    CC1101_CSN_LOW();
    HAL_Delay(1);
    CC1101_CSN_HIGH();
    HAL_Delay(1);

    if (CC1101Select() == CC1101_STATUS_OK)
        (void)CC1101ExchangeByte(CC1101_SRES, NULL);

    CC1101Deselect();
    HAL_Delay(1);
}

void CC1101SetIdle(void)
{
    TX_EN_LOW();
    RX_EN_LOW();
    CC1101WriteCmd(CC1101_SIDLE);
}

void CC1101ClrTXBuff(void)
{
    CC1101SetIdle();
    CC1101WriteCmd(CC1101_SFTX);
}

void CC1101ClrRXBuff(void)
{
    CC1101SetIdle();
    CC1101WriteCmd(CC1101_SFRX);
}

void CC1101SendPacket(uint8_t *txbuffer, uint8_t size, TX_DATA_MODE mode)
{
    uint8_t address = 0;

    if (txbuffer == NULL)
    {
        CC1101SetError(CC1101_STATUS_INVALID_ARGUMENT);
        return;
    }

    rxCatch = RESET;
    txFiFoUnFlow = RESET;

    if (mode == ADDRESS_CHECK)
        address = CC1101ReadReg(CC1101_ADDR);

    CC1101ClrTXBuff();

    if ((CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03U) != 0U)
    {
        CC1101WriteReg(CC1101_TXFIFO, (uint8_t)(size + 1U));
        CC1101WriteReg(CC1101_TXFIFO, address);
    }
    else
    {
        CC1101WriteReg(CC1101_TXFIFO, size);
    }

    if (size <= CC1101_TX_FIFO_CHUNK)
    {
        CC1101WriteMultiReg(CC1101_TXFIFO, txbuffer, size);
        CC1101SetTRMode(TX_MODE);

        if (!CC1101WaitItFlag(&rxCatch, SET, CC1101_STATUS_TX_TIMEOUT))
        {
            CC1101ClrTXBuff();
            return;
        }

        rxCatch = RESET;
    }
    else
    {
        CC1101WriteMultiReg(CC1101_TXFIFO, txbuffer, CC1101_TX_FIFO_CHUNK);
        CC1101SetTRMode(TX_MODE);

        for (uint8_t i = 0; i < (size / CC1101_TX_FIFO_CHUNK); i++)
        {
            if (!CC1101WaitItFlag(&txFiFoUnFlow, SET, CC1101_STATUS_TX_TIMEOUT))
            {
                CC1101ClrTXBuff();
                return;
            }

            txFiFoUnFlow = RESET;

            uint8_t offset = (uint8_t)((i + 1U) * CC1101_TX_FIFO_CHUNK);
            uint8_t remaining = (uint8_t)(size - offset);
            uint8_t chunk = remaining;

            if (chunk > CC1101_TX_FIFO_CHUNK)
                chunk = CC1101_TX_FIFO_CHUNK;

            if (chunk > 0U)
                CC1101WriteMultiReg(CC1101_TXFIFO, &txbuffer[offset], chunk);
        }
    }

    rxCatch = RESET;
    if (!CC1101WaitItFlag(&rxCatch, SET, CC1101_STATUS_TX_TIMEOUT))
    {
        CC1101ClrTXBuff();
        return;
    }

    rxCatch = RESET;
    CC1101ClrTXBuff();
}

uint8_t CC1101GetRXCnt(void)
{
    return (uint8_t)(CC1101ReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO);
}

void CC1101SetAddress(uint8_t address, ADDR_MODE AddressMode)
{
    uint8_t pktctrl1 = (uint8_t)(CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03U);

    CC1101WriteReg(CC1101_ADDR, address);

    if (AddressMode == BROAD_NO)
        pktctrl1 |= 0x01U;
    else if (AddressMode == BROAD_0)
        pktctrl1 |= 0x02U;
    else if (AddressMode == BROAD_0AND255)
        pktctrl1 |= 0x03U;

    CC1101WriteReg(CC1101_PKTCTRL1, pktctrl1);
}

void CC1101SetSYNC(uint16_t sync)
{
    CC1101WriteReg(CC1101_SYNC1, (uint8_t)(0xFFU & (sync >> 8)));
    CC1101WriteReg(CC1101_SYNC0, (uint8_t)(0xFFU & sync));
}

uint8_t CC1101RecPacket(uint8_t *rxBuffer, uint8_t *addr, uint8_t *rssi)
{
    uint8_t status[2];
    uint8_t pktLen;

    if (rxBuffer == NULL || addr == NULL || rssi == NULL)
    {
        CC1101SetError(CC1101_STATUS_INVALID_ARGUMENT);
        return 0;
    }

    if (CC1101GetRXCnt() == 0U)
        return 0;

    pktLen = CC1101ReadReg(CC1101_RXFIFO);

    if ((CC1101ReadReg(CC1101_PKTCTRL1) & ~0x03U) != 0U)
        *addr = CC1101ReadReg(CC1101_RXFIFO);

    if (pktLen == 0U)
        return 0;

    pktLen--;
    if (pktLen > _RECV_LENGTH)
    {
        CC1101ClrRXBuff();
        CC1101SetError(CC1101_STATUS_RX_OVERFLOW);
        return 0;
    }

    CC1101ReadMultiReg(CC1101_RXFIFO, rxBuffer, pktLen);
    CC1101ReadMultiReg(CC1101_RXFIFO, status, 2);
    *rssi = status[0];

    CC1101ClrRXBuff();

    if ((status[1] & CRC_OK) != 0U)
        return pktLen;

    CC1101SetError(CC1101_STATUS_CRC_ERROR);
    return 1;
}

void CC1101Init(uint8_t addr, uint16_t sync)
{
    CC1101_GDO_Init();
    CC1101Reset();

    for (uint8_t i = 0; i < CC1101_CONFIG_COUNT; i++)
        CC1101WriteReg(CC1101InitData[i][0], CC1101InitData[i][1]);

    CC1101SetAddress(addr, BROAD_0AND255);
    CC1101SetSYNC(sync);
    CC1101WriteMultiReg(CC1101_PATABLE, CC1101PaTable, sizeof(CC1101PaTable));

    rfid_printf("CC1101_PARTNUM = %d\n", CC1101ReadPartNumber());
    rfid_printf("CC1101_VERSION = %d\n", CC1101ReadVersion());
}

void CC1101_GDO_Init(void)
{
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE3);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE4);

    LL_GPIO_SetPinPull(GDO0_GPIO_Port, GDO0_Pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GDO2_GPIO_Port, GDO2_Pin, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GDO0_GPIO_Port, GDO0_Pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GDO2_GPIO_Port, GDO2_Pin, LL_GPIO_MODE_INPUT);

    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    NVIC_SetPriority(EXTI2_3_IRQn, 1);
    NVIC_EnableIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void CC1101_GDO_DeInit(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    GPIO_InitStruct.Pin = GDO0_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GDO0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GDO2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GDO2_GPIO_Port, &GPIO_InitStruct);

    NVIC_DisableIRQ(EXTI2_3_IRQn);
    NVIC_DisableIRQ(EXTI4_15_IRQn);
}

int16_t CC1101ReadRSSI(void)
{
    return CC1101CalcRSSI_dBm(CC1101ReadStatus(CC1101_RSSI));
}

int16_t CC1101CalcRSSI_dBm(uint8_t rssi_dec)
{
    const uint8_t rssi_offset = 74;

    if (rssi_dec >= 128U)
        return (int16_t)((int16_t)(rssi_dec - 256) / 2) - rssi_offset;

    return (int16_t)(rssi_dec / 2) - rssi_offset;
}
