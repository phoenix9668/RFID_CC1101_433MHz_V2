#ifndef ADXL362_PC_SPI_H
#define ADXL362_PC_SPI_H

#include <stdint.h>

#define SPI2 ((void *)0)

void SpiFunction(void *spi,
                 uint8_t *output,
                 uint8_t *input,
                 uint16_t output_size,
                 uint16_t input_size);

#endif
