/* Bench only. PB0 has no timer AF; the L0 GPIO bus cannot be driven by DMA. */
#include "main.h"
#include "sensor_test_clock.h"

static volatile uint32_t edge_count, max_latency;
static volatile bool late;
static uint32_t next_level, saved_systick_priority;
static bool running;

/* 32 MHz / 500 = 64 k edges/s = 32 kHz. Validate the actual waveform on D5. */
__attribute__((optimize("O2"))) void TIM2_IRQHandler(void)
{
    TIM2->SR = 0;
    GPIOB->BSRR = next_level;
    uint32_t latency = TIM2->CNT;
    next_level ^= UINT32_C(0x00010001);
    ++edge_count;
    if (latency > max_latency) max_latency = latency;
    if (latency > 200) late = true;
}

rfid_status_t sensor_test_clock_start(void)
{
    if (running || HAL_RCC_GetHCLKFreq() != 32000000 ||
        HAL_RCC_GetPCLK1Freq() != 32000000 || __get_PRIMASK())
        return RFID_INVALID;
    /* Caller has confirmed that the ADXL362 INT1 output is disabled. */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
    TIM2->PSC = 0;
    TIM2->ARR = 499;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CNT = 0;
    edge_count = max_latency = 0;
    late = false;
    next_level = LL_GPIO_PIN_0;
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    saved_systick_priority = NVIC_GetPriority(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, 3);
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    running = true;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
    return RFID_OK;
}

rfid_status_t sensor_test_clock_status(void)
{
    return !running ? RFID_INVALID : late ? RFID_TIMEOUT : RFID_OK;
}
sensor_test_clock_stats_t sensor_test_clock_stats(void)
{
    /* Individual aligned reads are atomic. Snapshot after stopping for final totals. */
    return (sensor_test_clock_stats_t){edge_count, max_latency, late};
}
void sensor_test_clock_stop(void)
{
    if (running)
    {
        NVIC_DisableIRQ(TIM2_IRQn);
        TIM2->DIER = 0;
        TIM2->CR1 = 0;
        NVIC_ClearPendingIRQ(TIM2_IRQn);
        LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);
        NVIC_SetPriority(SysTick_IRQn, saved_systick_priority);
        running = false;
    }
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);
}
