#ifndef ADXL362_PC_USART_H
#define ADXL362_PC_USART_H

#include <stdint.h>
#include <stdlib.h>

int pc_rfid_printf(const char *format, ...);
void HAL_Delay(uint32_t milliseconds);
void Error_Handler(void);

#define rfid_printf(...) pc_rfid_printf(__VA_ARGS__)

#endif
