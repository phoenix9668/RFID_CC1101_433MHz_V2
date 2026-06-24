#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cc1101.h"
#include "spi.h"
#include "usart.h"

uint8_t ErrorIndex;
int pc_verbose;

int pc_rfid_printf(const char *format, ...)
{
    if (!pc_verbose)
        return 0;

    va_list arguments;
    va_start(arguments, format);
    int result = vprintf(format, arguments);
    va_end(arguments);
    return result;
}

void SpiFunction(void *spi,
                 uint8_t *output,
                 uint8_t *input,
                 uint16_t output_size,
                 uint16_t input_size)
{
    (void)spi;
    (void)output;
    (void)output_size;
    if (input != NULL)
        memset(input, 0, input_size);
}

void HAL_Delay(uint32_t milliseconds)
{
    (void)milliseconds;
}

void Error_Handler(void)
{
}

void CC1101Send3AxisHandler(void)
{
}
