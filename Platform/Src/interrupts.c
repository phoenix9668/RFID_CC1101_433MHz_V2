#include "main.h"
#include "app.h"
#include "rtc.h"
void NMI_Handler(void) {}
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    for (;;)
    {
    }
}
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void)
{
    HAL_IncTick();
}
void RTC_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}
void EXTI0_1_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1))
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
        app_signal(APP_EVENT_FIFO);
    }
}
void EXTI2_3_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3))
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
        app_signal(APP_EVENT_RADIO);
    }
}
void EXTI4_15_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4))
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
        app_signal(APP_EVENT_RADIO);
    }
}
void USART1_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(USART1))
        (void)LL_USART_ReceiveData8(USART1);
    LL_USART_ClearFlag_ORE(USART1);
}
