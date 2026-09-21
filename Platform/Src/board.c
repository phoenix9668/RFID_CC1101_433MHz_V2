#include "board.h"
#include "app.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "rtc.h"
void SystemClock_Config(void);
static GPIO_TypeDef *pin_port(board_pin_t pin)
{
    return pin == PIN_SENSOR_CS || pin == PIN_RADIO_POWER || pin == PIN_RX_EN ? GPIOB : GPIOA;
}
static uint32_t pin_mask(board_pin_t pin)
{
    static const uint16_t masks[] = {1U << 2, 1U << 12, 1U << 6, 1U << 3,
                                     1U << 4, 1U << 5,  1U << 1, 1U << 2};
    return masks[pin];
}
void board_pin_write(board_pin_t pin, bool high)
{
    if (high)
        LL_GPIO_SetOutputPin(pin_port(pin), pin_mask(pin));
    else
        LL_GPIO_ResetOutputPin(pin_port(pin), pin_mask(pin));
}
bool board_pin_read(board_pin_t pin)
{
    return LL_GPIO_IsInputPinSet(pin_port(pin), pin_mask(pin)) != 0;
}
uint32_t board_millis(void)
{
    return HAL_GetTick();
}
uint32_t board_critical_enter(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}
void board_critical_exit(uint32_t saved)
{
    __set_PRIMASK(saved);
}
uint32_t board_rtc_millis(void)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
    uint32_t day = ((uint32_t)t.Hours * 3600 + (uint32_t)t.Minutes * 60 + t.Seconds) * 1000;
    day += (t.SecondFraction - t.SubSeconds) * 1000 / (t.SecondFraction + 1);
    static uint32_t previous, monotonic;
    static bool started;
    if (!started)
    {
        previous = day;
        started = true;
    }
    monotonic += day >= previous ? day - previous : 86400000U - previous + day;
    previous = day;
    return monotonic;
}
void board_delay(uint32_t ms)
{
    HAL_Delay(ms);
}
void board_watchdog(void)
{
    LL_IWDG_ReloadCounter(IWDG);
}
static void analog_pins(GPIO_TypeDef *port, uint32_t mask)
{
    LL_GPIO_InitTypeDef config = {0};
    config.Pin = mask;
    config.Mode = LL_GPIO_MODE_ANALOG;
    config.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(port, &config);
}
void board_spi_enable(unsigned bus, bool enabled)
{
    SPI_TypeDef *spi = bus == 1 ? SPI1 : SPI2;
    if (enabled)
    {
        if (bus == 1)
        {
            MX_SPI1_Init();
            LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
            LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);
            LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3 | LL_EXTI_LINE_4);
            LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_3);
            LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_3 | LL_EXTI_LINE_4);
            LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3 | LL_EXTI_LINE_4);
            NVIC_EnableIRQ(EXTI2_3_IRQn);
            NVIC_EnableIRQ(EXTI4_15_IRQn);
            board_pin_write(PIN_RADIO_CS, true);
        }
        else
        {
            MX_SPI2_Init();
            board_pin_write(PIN_SENSOR_CS, true);
        }
        return;
    }
    LL_SPI_Disable(spi);
    LL_SPI_DeInit(spi);
    if (bus == 1)
    {
        LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1);
        analog_pins(GPIOA, LL_GPIO_PIN_2 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    }
    else
    {
        LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2);
        analog_pins(GPIOB, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
        /* Sensor stays powered: its CS must never float in STOP. */
        board_pin_write(PIN_SENSOR_CS, true);
    }
}
rfid_status_t board_spi_exchange(unsigned bus, uint8_t *data, size_t count)
{
    if (!data || !count || count > 512 || (bus != 1 && bus != 2))
        return RFID_INVALID;
    SPI_TypeDef *spi = bus == 1 ? SPI1 : SPI2;
    uint32_t deadline = HAL_GetTick() + 5;
    unsigned budget = 500000;
    for (size_t i = 0; i < count; ++i)
    {
        while (!LL_SPI_IsActiveFlag_TXE(spi))
            if ((int32_t)(HAL_GetTick() - deadline) >= 0 || !--budget)
                return RFID_TIMEOUT;
        LL_SPI_TransmitData8(spi, data[i]);
        while (!LL_SPI_IsActiveFlag_RXNE(spi))
            if ((int32_t)(HAL_GetTick() - deadline) >= 0 || !--budget)
                return RFID_TIMEOUT;
        data[i] = LL_SPI_ReceiveData8(spi);
        if (LL_SPI_IsActiveFlag_OVR(spi) || LL_SPI_IsActiveFlag_MODF(spi))
            return RFID_IO;
    }
    while (LL_SPI_IsActiveFlag_BSY(spi))
        if ((int32_t)(HAL_GetTick() - deadline) >= 0 || !--budget)
            return RFID_TIMEOUT;
    return RFID_OK;
}
void board_radio_off(void)
{
    board_pin_write(PIN_TX_EN, false);
    board_pin_write(PIN_RX_EN, false);
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_3 | LL_EXTI_LINE_4);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3 | LL_EXTI_LINE_4);
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
    /* Stop driving all unpowered module inputs before dropping its supply. */
    board_spi_enable(1, false);
    analog_pins(GPIOA, LL_GPIO_PIN_3 | LL_GPIO_PIN_4);
    board_pin_write(PIN_RADIO_POWER, false);
}
void board_init(void)
{
    MX_GPIO_Init();
    board_radio_off();
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_12, LL_GPIO_PULL_NO);
    /* No clock output or unused digital input buffers in production. */
    analog_pins(GPIOA, LL_GPIO_PIN_8 | LL_GPIO_PIN_11 | LL_GPIO_PIN_15);
    analog_pins(GPIOB, LL_GPIO_PIN_0 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_6 |
                           LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 |
                           LL_GPIO_PIN_11);
#if RFID_DIAGNOSTICS
    MX_USART1_UART_Init();
#else
    analog_pins(GPIOA, LL_GPIO_PIN_9 | LL_GPIO_PIN_10);
#endif
    /* ARMv6-M DHCSR.C_DEBUGEN; this CMSIS M0+ header omits CoreDebug. */
    if (!(*(volatile const uint32_t *)UINT32_C(0xe000edf0) & 1U))
    {
        HAL_DBGMCU_DisableDBGStopMode();
        HAL_DBGMCU_DisableDBGStandbyMode();
        __HAL_DBGMCU_UNFREEZE_IWDG();
        __HAL_DBGMCU_UNFREEZE_RTC();
    }
}
void board_idle(bool allow_stop)
{
    if (!allow_stop)
    {
        __WFI();
        return;
    }
    uint32_t delay = app_wakeup_delay_ms();
    if (!delay)
        return;
    /* Re-arm before masking IRQs: HAL's timeout needs the running SysTick.
       Ceiling division prevents waking just before an application deadline. */
    uint32_t ticks = (delay * 2048U + 999U) / 1000U;
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks - 1, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!app_events_pending())
    {
        HAL_SuspendTick();
        __HAL_RCC_PWR_CLK_ENABLE();
        HAL_PWREx_EnableUltraLowPower();
        HAL_PWREx_EnableFastWakeUp();
        __DSB();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        /* IRQs only post events. Restore a working tick before HAL clock timeouts. */
        SystemCoreClockUpdate();
        HAL_InitTick(TICK_INT_PRIORITY);
        __set_PRIMASK(primask);
        SystemClock_Config();
        return;
    }
    __set_PRIMASK(primask);
}
uint32_t board_eeprom_read(void *context, uint16_t offset)
{
    (void)context;
    if (offset > 0x7fc || (offset & 3))
        return UINT32_MAX;
    return *(volatile uint32_t *)(uintptr_t)(EEPROM_START_ADDR + offset);
}
rfid_status_t board_eeprom_write(void *context, uint16_t offset, uint32_t value)
{
    (void)context;
    if (offset < 0x240 || offset > 0x7fc || (offset & 3))
        return RFID_INVALID;
    uint32_t address = EEPROM_START_ADDR + offset;
    if (*(volatile uint32_t *)(uintptr_t)address == value)
        return RFID_OK;
    if (HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK)
        return RFID_IO;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR |
                           FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);
    /* Fixed-time programming performs erase/write; no redundant explicit erase. */
    HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();
    HAL_StatusTypeDef status =
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, value);
    HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram();
    HAL_FLASHEx_DATAEEPROM_Lock();
    if (status != HAL_OK || *(volatile uint32_t *)(uintptr_t)address != value)
        return RFID_IO;
    return RFID_OK;
}
rfid_status_t board_battery(uint16_t *value)
{
    if (!value)
        return RFID_INVALID;
    MX_ADC_Init();
    rfid_status_t result = RFID_OK;
    uint32_t sum = 0;
    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
        result = RFID_IO;
    for (unsigned i = 0; i < 11 && result == RFID_OK; ++i)
    {
        if (HAL_ADC_Start(&hadc) != HAL_OK || HAL_ADC_PollForConversion(&hadc, 5) != HAL_OK)
        {
            result = RFID_TIMEOUT;
            break;
        }
        uint16_t sample = (uint16_t)HAL_ADC_GetValue(&hadc);
        if (i)
            sum += sample;
        HAL_ADC_Stop(&hadc);
        HAL_Delay(1);
    }
    HAL_ADC_DeInit(&hadc);
    if (result == RFID_OK)
        *value = (uint16_t)(sum / 10);
    return result;
}
void board_identity(uint8_t id[6], uint64_t *seed, uint64_t *stream)
{
    uint32_t a = board_eeprom_read(NULL, 0), b = board_eeprom_read(NULL, 4);
    id[0] = a >> 24;
    id[1] = a >> 16;
    id[2] = a >> 8;
    id[3] = (uint8_t)a;
    id[4] = b >> 24;
    id[5] = b >> 16;
    *seed = (uint64_t)HAL_GetUIDw0() << 32 | HAL_GetUIDw1();
    *stream = (uint64_t)HAL_GetUIDw2() << 32 | a;
}
void board_log(const char *message)
{
#if RFID_DIAGNOSTICS
    uint32_t end = HAL_GetTick() + 20;
    while (*message)
    {
        while (!LL_USART_IsActiveFlag_TXE(USART1))
            if ((int32_t)(HAL_GetTick() - end) >= 0)
                return;
        LL_USART_TransmitData8(USART1, (uint8_t)*message++);
    }
    while (!LL_USART_IsActiveFlag_TC(USART1))
        if ((int32_t)(HAL_GetTick() - end) >= 0)
            return;
#else
    (void)message;
#endif
}
