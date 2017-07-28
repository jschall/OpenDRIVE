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

static void led_spi_send_byte(uint8_t byte) {
    spi_send8(SPI3, byte);
    while (SPI_SR(SPI3) & SPI_SR_BSY);
}

static void spi_init(void) {
    rcc_periph_clock_enable(RCC_SPI3);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15); // LED CS
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5); // MS5611 nCS
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // ICM nCS
    gpio_clear(GPIOA, GPIO15); // LED CS down
    gpio_set(GPIOB, GPIO0); // ICM nCS up
    gpio_set(GPIOA, GPIO5); // MS5611 nCS up

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4|GPIO5); // MISO,MOSI
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3); // SCK
    gpio_set_af(GPIOB, GPIO_AF6, GPIO3|GPIO4|GPIO5);

    spi_set_full_duplex_mode(SPI3);
    spi_set_unidirectional_mode(SPI3);
    spi_enable_software_slave_management(SPI3);
    spi_set_nss_high(SPI3);
    spi_set_master_mode(SPI3);
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

static void icm_init(void) {
    // setup SPI for ICM
    spi_disable(SPI3);
    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_16); // <7mhz
    spi_set_clock_polarity_1(SPI3);
    spi_set_clock_phase_1(SPI3);
    spi_set_data_size(SPI3, SPI_CR2_DS_16BIT);
    spi_send_msb_first(SPI3);
    spi_fifo_reception_threshold_16bit(SPI3);
    spi_enable(SPI3);


    gpio_clear(GPIOB, GPIO0); // ICM nCS down
    __asm__("nop"); // min 7ns

    uint16_t cmd, ret;

    cmd = (0x80|0x00)<<8; // WHOAMI
    ret = spi_xfer(SPI3, cmd);

    // bring ICM out of sleep and use internal oscillator
    cmd = (0x06<<8)|0x00;
    spi_xfer(SPI3,cmd);

    // sleep 10us
    usleep(10);

    cmd = (0x03<<8)|(1<<5); // I2C master enable
    spi_xfer(SPI3,cmd);

    cmd = (0x0f<<8)|(1<<1); // BYPASS_EN = 1
    spi_xfer(SPI3,cmd);

    // set up ICM's I2C master
    cmd = (0x7F<<8)|3; // user bank 3
    spi_xfer(SPI3,cmd);

    cmd = (0x01<<8)|7; // I2C_MST_CLK = 7 per table 23
    spi_xfer(SPI3,cmd);

//     cmd = (0x02<<8)|(1<<7); // DELAY_ES_SHADOW = 1
//     spi_xfer(SPI3,cmd);

    cmd = (0x03<<8)|0x0C|0x80; // I2C_ID_0 = 0x0C, read
    spi_xfer(SPI3,cmd);

    cmd = (0x04<<8)|0x01; // I2C_SLV0_REG = 0x01
    spi_xfer(SPI3,cmd);

    cmd = (0x05<<8)|(1<<0)|(1<<7); // I2C_SLV0_LENG = 1, I2C_SLV0_EN = 1
    spi_xfer(SPI3,cmd);

    cmd = (0x7F<<8)|0; // user bank 0
    spi_xfer(SPI3,cmd);

    for(uint8_t i=0;i<36;i++) __asm__("nop"); // min 500ns
    gpio_set(GPIOB, GPIO0); // ICM nCS up
    for(uint8_t i=0;i<4;i++) __asm__("nop"); // min 50ns

    usleep(100000);

    gpio_clear(GPIOB, GPIO0); // ICM nCS down
    __asm__("nop"); // min 7ns

    cmd = (0x80|0x03)<<8;
    ret = spi_xfer(SPI3,cmd);

    char temp[33];
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "AK0", itoa(ret,temp,16));

    cmd = (0x80|0x17)<<8; // I2C_MST_STATUS
    ret = spi_xfer(SPI3, cmd);
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "AK1", itoa(ret,temp,16));

    cmd = (0x80|0x3B)<<8; // EXT_SLV_SENS_DATA_00
    ret = spi_xfer(SPI3, cmd);
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "AK2", itoa(ret,temp,16));

    for(uint8_t i=0;i<36;i++) __asm__("nop"); // min 500ns
    gpio_set(GPIOB, GPIO0); // ICM nCS up
    for(uint8_t i=0;i<4;i++) __asm__("nop"); // min 50ns

}

static void icm_update(void) {

}

static void led_update(void) {
    struct led_color_s led_colors[4];
    uint8_t num_leds = 4;
    for(uint8_t i=0; i<num_leds; i++) {
        led_make_brg_color_rgb(led_r,led_g,led_b,&led_colors[i]);
    }

    // setup SPI for LED
    spi_disable(SPI3);
    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_32);
    spi_set_clock_polarity_0(SPI3);
    spi_set_clock_phase_0(SPI3);
    spi_set_data_size(SPI3, SPI_CR2_DS_8BIT);
    spi_send_msb_first(SPI3);
    spi_fifo_reception_threshold_8bit(SPI3);
    spi_enable(SPI3);

    gpio_set(GPIOA, GPIO15);
    for(uint8_t i=0;i<30;i++) __asm__("nop");
    led_write(num_leds, led_colors, led_spi_send_byte);
    led_spi_send_byte(0); // seems to be required for last LED to set color
    for(uint8_t i=0;i<30;i++) __asm__("nop");
    gpio_clear(GPIOA, GPIO15);
}

static void uavcan_ready_handler(void) {
    icm_init();
    icm_update();
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

    spi_init();
    i2c_slave_init();

    uint32_t tprint_us = 0;
    // main loop
    while(1) {
        uavcan_update();

        uint32_t tnow_us = micros();
        if (tnow_us-tprint_us > 8333) {
            led_update();
            tprint_us = tnow_us;
        }

        if (restart_req && (micros() - restart_req_us) > 1000) {
            union shared_msg_payload_u msg;
            fill_shared_canbus_info(&msg.canbus_info);
            shared_msg_finalize_and_write(SHARED_MSG_CANBUS_INFO, &msg);

            scb_reset_system();
        }
    }

    return 0;
}
