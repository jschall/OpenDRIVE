#include "spi.h"

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static enum spi_device_t current_spi_device;

static void spi_assert_chip_select(enum spi_device_t spi_device) {
    switch(spi_device) {
        case SPI_DEVICE_UBLED:
            gpio_set(GPIOA, GPIO15);
            break;
        case SPI_DEVICE_ICM:
            gpio_clear(GPIOB, GPIO0);
            break;
        case SPI_DEVICE_MS5611:
            gpio_clear(GPIOA, GPIO5);
            break;
        default:
            break;
    }
}

static void spi_deassert_chip_select(enum spi_device_t spi_device) {
    switch(spi_device) {
        case SPI_DEVICE_UBLED:
            gpio_clear(GPIOA, GPIO15);
            break;
        case SPI_DEVICE_ICM:
            gpio_set(GPIOB, GPIO0);
            break;
        case SPI_DEVICE_MS5611:
            gpio_set(GPIOA, GPIO5);
            break;
        default:
            break;
    }
}

void spi_begin(enum spi_device_t spi_device) {
    if (spi_device == current_spi_device) {
        spi_assert_chip_select(spi_device);
        return;
    }

    if (current_spi_device == SPI_DEVICE_UNINITIALIZED) {
        rcc_periph_clock_enable(RCC_SPI3);
        rcc_periph_clock_enable(RCC_GPIOA);
        rcc_periph_clock_enable(RCC_GPIOB);
        gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15); // LED CS
        gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5); // MS5611 nCS
        gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // ICM nCS

        gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4|GPIO5); // MISO,MOSI
        gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
        gpio_set_af(GPIOB, GPIO_AF6, GPIO3|GPIO4|GPIO5);
    }

    while ((SPI_SR(SPI3) & (0b11<<11)) != 0); // wait for TXFIFO to be empty
    while (SPI_SR(SPI3) & SPI_SR_BSY); // wait for bsy

    for (uint8_t i=1; i<NUM_SPI_DEVICES; i++) {
        spi_deassert_chip_select(i);
    }

    spi_reset(SPI3);

    spi_set_full_duplex_mode(SPI3);
    spi_set_unidirectional_mode(SPI3);
    spi_enable_software_slave_management(SPI3);
    spi_set_nss_high(SPI3);
    spi_set_master_mode(SPI3);
    spi_send_msb_first(SPI3);

    switch(spi_device) {
        case SPI_DEVICE_UBLED:
            spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_16); //<30mhz
            spi_set_clock_polarity_0(SPI3);
            spi_set_clock_phase_0(SPI3);
            spi_set_data_size(SPI3, SPI_CR2_DS_8BIT);
            spi_fifo_reception_threshold_8bit(SPI3);
            break;
        case SPI_DEVICE_ICM:
            spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_8); // <7mhz
            spi_set_clock_polarity_1(SPI3);
            spi_set_clock_phase_1(SPI3);
            spi_set_data_size(SPI3, SPI_CR2_DS_16BIT);
            spi_fifo_reception_threshold_16bit(SPI3);
            break;
        default:
            break;
    }

    spi_enable(SPI3);
    spi_assert_chip_select(spi_device);
    current_spi_device = spi_device;
}

void spi_end(void) {
    while ((SPI_SR(SPI3) & (0b11<<11)) != 0); // wait for TXFIFO to be empty
    while (SPI_SR(SPI3) & SPI_SR_BSY); // wait for bsy
    spi_deassert_chip_select(current_spi_device);
}
