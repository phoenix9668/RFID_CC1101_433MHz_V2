#ifndef LEGACY_STM32_STUB_H
#define LEGACY_STM32_STUB_H
#include <stdint.h>
#include <stdlib.h>
#define __IO volatile
typedef enum { RESET = 0, SET = 1 } FlagStatus;
#define _Original_Data_Algorithm 1
#define rfid_printf(...) ((void)0)
extern uint8_t ErrorIndex;
static inline void HAL_Delay(uint32_t n) { (void)n; }
static inline void Error_Handler(void) { abort(); }
#endif
