/* STM32L051C8 startup. Application-owned logic lives outside generated files. */
#include "main.h"
#include "board.h"
#include "app.h"
#include "iwdg.h"
#include "rtc.h"
#if RFID_SENSOR_ODR_TEST
#include "sensor_odr_test.h"
#endif
#if RFID_DIAGNOSTIC_HOLD_AWAKE
#include "sensor.h"
#include <stdio.h>
static void trace_timing(void)
{
    uint8_t filter = 0, power = 0;
    rfid_status_t a = sensor_read_register(0x2c, &filter);
    rfid_status_t b = sensor_read_register(0x2d, &power);
    char line[160];
    snprintf(line, sizeof(line),
             "timing tick=%lu rtc=%lu cr=%08lX wutr=%lu imr=%08lX filter=%02X power=%02X io=%u/%u\r\n",
             (unsigned long)board_millis(), (unsigned long)board_rtc_millis(),
             (unsigned long)RTC->CR, (unsigned long)RTC->WUTR,
             (unsigned long)EXTI->IMR, filter, power, a, b);
    board_log(line);
}
#endif
void SystemClock_Config(void);
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    board_init();
    MX_IWDG_Init();
    MX_RTC_Init();
#if RFID_SENSOR_ODR_TEST
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    NVIC_DisableIRQ(EXTI0_1_IRQn);
    (void)sensor_odr_test_run(240000);
    for (;;)
    {
        board_watchdog();
        __WFI();
    }
#else
    app_init();
#if RFID_DIAGNOSTIC_HOLD_AWAKE
    board_log("bench hold-awake: STOP disabled\r\n");
    uint32_t next_trace = board_millis();
#endif
    for (;;)
    {
#if RFID_DIAGNOSTIC_HOLD_AWAKE
        (void)app_poll();
        if ((int32_t)(board_millis() - next_trace) >= 0)
        {
            trace_timing();
            next_trace = board_millis() + 10000;
        }
        board_idle(false);
#else
        board_idle(app_poll());
#endif
    }
#endif
}
void Error_Handler(void)
{
    __disable_irq();
    /* Fatal clock/configuration errors reset the MCU; peripheral errors return. */
    NVIC_SystemReset();
    for (;;)
    {
    }
}
