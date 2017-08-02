#pragma once

#include <libopencm3/stm32/spi.h>

enum spi_device_t {
    SPI_DEVICE_UNINITIALIZED = 0,
    SPI_DEVICE_UBLED,
    SPI_DEVICE_ICM,
    SPI_DEVICE_MS5611,
    NUM_SPI_DEVICES
};

void spi_begin(enum spi_device_t spi_device);
void spi_end(void);
