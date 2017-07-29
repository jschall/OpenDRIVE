/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include "timing.h"
#include "init.h"
#include "can.h"
#include "uavcan.h"
#include "flash.h"
#include <libopencm3/cm3/scb.h>
#include <string.h>
#include <bootloader/shared.h>
#include <stdlib.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <math.h>
#include "led.h"


#define CANBUS_AUTOBAUD_SWITCH_INTERVAL_US 1000100 // not exactly one second, to avoid potential race conditions
#define CANBUS_AUTOBAUD_TIMEOUT_US 10000000

static volatile const struct shared_app_descriptor_s shared_app_descriptor __attribute__((section(".app_descriptor"),used)) = {
    .signature = SHARED_APP_DESCRIPTOR_SIGNATURE,
    .image_crc = 0,
    .image_size = 0,
    .vcs_commit = GIT_HASH,
    .major_version = 1,
    .minor_version = 0,
    .boot_delay_sec = APP_CONFIG_BOOT_DELAY_SEC,
    .canbus_disable_auto_baud = !APP_CONFIG_CAN_AUTO_BAUD_ENABLE,
    .canbus_baudrate = APP_CONFIG_CAN_DEFAULT_BAUDRATE,
    .canbus_local_node_id = APP_CONFIG_CAN_LOCAL_NODE_ID
};

static bool restart_req = false;
static uint32_t restart_req_us;

static enum shared_msg_t shared_msgid;
static union shared_msg_payload_u shared_msg;
static bool shared_msg_valid;

static void fill_shared_canbus_info(struct shared_canbus_info_s* canbus_info) {
    canbus_info->local_node_id = uavcan_get_node_id();

    if (canbus_get_confirmed_baudrate()) {
        canbus_info->baudrate = canbus_get_confirmed_baudrate();
    } else if (shared_msg_valid && canbus_baudrate_valid(shared_msg.canbus_info.baudrate)) {
        canbus_info->baudrate = canbus_info->baudrate;
    } else {
        canbus_info->baudrate = 0;
    }
}

static void set_uavcan_node_info(void)
{
    struct uavcan_node_info_s uavcan_node_info;
    memset(&uavcan_node_info, 0, sizeof(uavcan_node_info));

    if (shared_msg_valid && shared_msgid == SHARED_MSG_BOOT_INFO && shared_msg.boot_info_msg.hw_info) {
        uavcan_node_info.hw_name = shared_msg.boot_info_msg.hw_info->hw_name;
        uavcan_node_info.hw_major_version = shared_msg.boot_info_msg.hw_info->hw_major_version;
        uavcan_node_info.hw_minor_version = shared_msg.boot_info_msg.hw_info->hw_minor_version;
    }

    uavcan_node_info.sw_major_version = shared_app_descriptor.major_version;
    uavcan_node_info.sw_minor_version = shared_app_descriptor.minor_version;
    uavcan_node_info.sw_vcs_commit_available = true;
    uavcan_node_info.sw_vcs_commit = shared_app_descriptor.vcs_commit;
    uavcan_node_info.sw_image_crc_available = true;
    uavcan_node_info.sw_image_crc = shared_app_descriptor.image_crc;

    uavcan_set_node_info(uavcan_node_info);
}


static bool restart_request_handler(void)
{
    restart_req = true;
    restart_req_us = micros();
    return true;
}

static void file_beginfirmwareupdate_handler(struct uavcan_transfer_info_s transfer_info, uint8_t source_node_id, const char* path)
{
    if (source_node_id == 0) {
        source_node_id = transfer_info.remote_node_id;
    }

    union shared_msg_payload_u msg;
    fill_shared_canbus_info(&msg.firmwareupdate_msg.canbus_info);

    if (source_node_id > 0 && source_node_id <= 127) {
        msg.firmwareupdate_msg.source_node_id = source_node_id;
    } else {
        msg.firmwareupdate_msg.source_node_id = 0;
    }

    strcpy(msg.firmwareupdate_msg.path, path);

    shared_msg_finalize_and_write(SHARED_MSG_FIRMWAREUPDATE, &msg);

    uavcan_send_file_beginfirmwareupdate_response(&transfer_info, UAVCAN_BEGINFIRMWAREUPDATE_ERROR_OK, "");

    uint32_t tbegin_us = micros();
    while (micros()-tbegin_us < 100000) {
        uavcan_update();
    }

    scb_reset_system();
}

static void i2c_slave_init(void) {
    rcc_periph_clock_enable(RCC_I2C2);
    i2c_reset(I2C2);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9|GPIO10);
    gpio_set_af(GPIOA, GPIO_AF4, GPIO9|GPIO10);
    i2c_peripheral_disable(I2C2);
    i2c_enable_analog_filter(I2C2);
    i2c_set_digital_filter(I2C2, I2C_CR1_DNF_DISABLED);
    rcc_set_i2c_clock_sysclk(I2C2);
    i2c_set_prescaler(I2C2,8);
    i2c_set_data_setup_time(I2C2,9);
    i2c_set_data_hold_time(I2C2,11);
    i2c_enable_stretching(I2C2);
    i2c_set_7bit_addr_mode(I2C2);
    I2C_OAR1(I2C2) = (0x55&0xFF) << 1;
    I2C_OAR1(I2C2) |= (1<<15);
    nvic_enable_irq(NVIC_I2C2_EV_EXTI24_IRQ);
    I2C_CR1(I2C2) |= (1<<2); // RXIE
    I2C_CR1(I2C2) |= (1<<3); // ADDRIE
    i2c_peripheral_enable(I2C2);
}

static uint8_t led_r,led_g,led_b;
static uint8_t led_reg;
static uint8_t i2c2_recv_byte_idx;
static uint8_t i2c2_transfer_address;

static void toshibaled_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte) {
    if (recv_byte_idx == 0) {
        led_reg = recv_byte;
    } else {
        switch(led_reg) {
            case 1:
                led_b = ((recv_byte << 4)&0xf0) | (recv_byte&0x0f);
                break;
            case 2:
                led_g = ((recv_byte << 4)&0xf0) | (recv_byte&0x0f);
                break;
            case 3:
                led_r = ((recv_byte << 4)&0xf0) | (recv_byte&0x0f);
                break;
        }
        led_reg++;
    }
}

void i2c2_ev_exti24_isr(void) {
    if (I2C_ISR(I2C2) & (1<<3)) { // ADDR
        i2c2_transfer_address = (I2C_ISR(I2C2) >> 17) & 0x7FU; // ADDCODE
        i2c2_recv_byte_idx = 0;
        I2C_ICR(I2C2) |= (1<<3);
    }

    if (i2c_received_data(I2C2)) {
        uint8_t recv_byte = i2c_get_data(I2C2); // reading clears our interrupt flag
        switch(i2c2_transfer_address) {
            case 0x55:
                toshibaled_interface_recv_byte(i2c2_recv_byte_idx, recv_byte);
                break;
        }
        i2c2_recv_byte_idx++;
    }
}

enum spi_device_t {
    SPI_DEVICE_UNINITIALIZED = 0,
    SPI_DEVICE_UBLED,
    SPI_DEVICE_ICM,
    SPI_DEVICE_MS5611,
    NUM_SPI_DEVICES
};

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

static void spi_begin(enum spi_device_t spi_device) {
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

static void spi_end(void) {
    while ((SPI_SR(SPI3) & (0b11<<11)) != 0); // wait for TXFIFO to be empty
    while (SPI_SR(SPI3) & SPI_SR_BSY); // wait for bsy
    spi_deassert_chip_select(current_spi_device);
}

static uint16_t icm_spi_transfer(uint16_t data) {
    spi_begin(SPI_DEVICE_ICM);

    SPI_DR(SPI3) = data;
    while (!(SPI_SR(SPI3) & SPI_SR_RXNE));
    uint16_t ret = SPI_DR(SPI3);

    spi_end();
    return ret;
}

#include "icm_defines.h"

static int8_t icm_user_bank = 0;

static void icm_set_user_bank(uint8_t user_bank) {
    if (user_bank != icm_user_bank) {
        icm_spi_transfer((ICM20948_REG_BANK_SEL<<8)|(user_bank<<4));
        icm_user_bank = user_bank;
    }
}

static uint16_t icm_read_reg(uint8_t user_bank, uint8_t reg) {
    icm_set_user_bank(user_bank);

    return icm_spi_transfer(((uint16_t)reg|0x80)<<8);
}

static void icm_write_reg(uint8_t user_bank, uint8_t reg, uint8_t value) {
    icm_set_user_bank(user_bank);
    icm_spi_transfer((reg<<8)|value);
}

static void icm_init(void) {
//     volatile uint16_t user_ctrl = icm_read_reg(ICM20948_USER_CTRL);
//     icm_write_reg(ICM20948_USER_CTRL, user_ctrl&~(1<<5)); // I2C_MST_EN
    usleep(10000);
    icm_write_reg(ICM20948_PWR_MGMT_1, 1<<7); // DEVICE_RESET
    usleep(100000);
    icm_write_reg(ICM20948_PWR_MGMT_1, 1);
    usleep(15000);
//     usleep(100000);
//     icm_write_reg(ICM20948_INT_PIN_CFG, 1<<1);
    icm_write_reg(ICM20948_USER_CTRL, (1<<4)|(1<<5)); // disable i2c interface, enable i2c master
    usleep(10000);

    uint8_t user_ctrl = icm_read_reg(ICM20948_USER_CTRL);

    icm_write_reg(ICM20948_PWR_MGMT_1, 1);
    usleep(15000);
    icm_write_reg(ICM20948_PWR_MGMT_2, 0);

    icm_write_reg(ICM20948_I2C_MST_CTRL, (1<<4)|7);

    icm_write_reg(ICM20948_I2C_SLV0_CTRL, 0);
    icm_write_reg(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR|0x80);
    icm_write_reg(ICM20948_I2C_SLV0_REG, 0);
    icm_write_reg(ICM20948_I2C_SLV0_CTRL, 0x80|2);

    usleep(100000);
    uint8_t whoami = icm_read_reg(ICM20948_WHO_AM_I);
    uint8_t data00 = icm_read_reg(ICM20948_EXT_SLV_SENS_DATA_00);
    uint8_t data01 = icm_read_reg(ICM20948_EXT_SLV_SENS_DATA_01);

    char temp[33];
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "AK0", itoa(whoami,temp,16));
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "AK1", itoa(user_ctrl,temp,16));
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "AK2", itoa(data00,temp,16));
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "AK3", itoa(data01,temp,16));

}

static void icm_update(void) {
}

static void led_spi_send_byte(uint8_t byte) {
    while (!(SPI_SR(SPI3) & SPI_SR_TXE)); // wait for bus to become available
    SPI_DR8(SPI3) = byte;
}

static void led_update(void) {
    static uint32_t last_update_us;
    uint32_t tnow_us = micros();
    if (tnow_us-last_update_us < 8333) {
        return;
    }
    last_update_us = tnow_us;

#if 0
    led_r = (millis()/1000)%3 == 0 ? 255 : 0;
    led_g = (millis()/1000)%3 == 1 ? 255 : 0;
    led_b = (millis()/1000)%3 == 2 ? 255 : 0;
#endif


    struct led_color_s led_colors[4];
    uint8_t num_leds = 4;
    for(uint8_t i=0; i<num_leds; i++) {
        led_make_brg_color_rgb(led_r,led_g,led_b,&led_colors[i]);
    }

    spi_begin(SPI_DEVICE_UBLED);
    for(uint8_t i=0;i<30;i++) __asm__("nop");
    led_write(num_leds, led_colors, led_spi_send_byte);
    led_spi_send_byte(0); // seems to be required for last LED to set color
    for(uint8_t i=0;i<30;i++) __asm__("nop");

    spi_end();
}

static void uavcan_ready_handler(void) {
    icm_init();
}

static uint32_t get_canbus_baud(void) {
    uint32_t initial_canbus_baud;
    if (shared_msg_valid && canbus_baudrate_valid(shared_msg.canbus_info.baudrate)) {
        initial_canbus_baud = shared_msg.canbus_info.baudrate;
    } else if (canbus_baudrate_valid(shared_app_descriptor.canbus_baudrate)) {
        initial_canbus_baud = shared_app_descriptor.canbus_baudrate;
    } else {
        initial_canbus_baud = 1000000;
    }

    bool canbus_autobaud_enable;
    if (shared_msg_valid && canbus_baudrate_valid(shared_msg.canbus_info.baudrate)) {
        canbus_autobaud_enable = false;
    } else if (shared_app_descriptor.canbus_disable_auto_baud) {
        canbus_autobaud_enable = false;
    } else {
        canbus_autobaud_enable = true;
    }

    uint32_t canbus_baud = initial_canbus_baud;
    uint32_t autobaud_begin_us = micros();
    if (canbus_autobaud_enable) {
        struct canbus_autobaud_state_s autobaud_state;

        canbus_autobaud_start(&autobaud_state, canbus_baud, CANBUS_AUTOBAUD_SWITCH_INTERVAL_US);

        while (!(autobaud_state.success || micros()-autobaud_begin_us > CANBUS_AUTOBAUD_TIMEOUT_US)) {
            canbus_baud = canbus_autobaud_update(&autobaud_state);
        }

        if (!autobaud_state.success) {
            canbus_baud = initial_canbus_baud;
        }
    }
    return canbus_baud;
}

int main(void)
{
    init_clock();
    timing_init();

    shared_msg_valid = shared_msg_check_and_retreive(&shared_msgid, &shared_msg);
    shared_msg_clear();

    canbus_init(get_canbus_baud(), false);
    uavcan_init();

    set_uavcan_node_info();
    uavcan_set_uavcan_ready_cb(uavcan_ready_handler);
    uavcan_set_restart_cb(restart_request_handler);
    uavcan_set_file_beginfirmwareupdate_cb(file_beginfirmwareupdate_handler);
    uavcan_set_node_mode(UAVCAN_MODE_OPERATIONAL);

    if (shared_msg_valid && shared_msg.canbus_info.local_node_id > 0 && shared_msg.canbus_info.local_node_id <= 127) {
        uavcan_set_node_id(shared_msg.canbus_info.local_node_id);
    } else if (shared_app_descriptor.canbus_local_node_id > 0 && shared_app_descriptor.canbus_local_node_id <= 127) {
        uavcan_set_node_id(shared_msg.canbus_info.local_node_id);
    }

    i2c_slave_init();

    // main loop
    while(1) {
        uavcan_update();
        icm_update();
        led_update();

        if (restart_req && (micros() - restart_req_us) > 1000) {
            union shared_msg_payload_u msg;
            fill_shared_canbus_info(&msg.canbus_info);
            shared_msg_finalize_and_write(SHARED_MSG_CANBUS_INFO, &msg);

            scb_reset_system();
        }
    }

    return 0;
}
