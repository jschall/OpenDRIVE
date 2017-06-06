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
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

static bool restart_req = false;
static uint32_t restart_req_us = 0;

static bool restart_request_handler(void)
{
    restart_req = true;
    restart_req_us = micros();
    return true;
}

static uint16_t flow_read_register(uint8_t reg)
{
    uint8_t i;
    gpio_clear(GPIOA, GPIO15); // assert FLOW chip select
    for(i=0;i<12;i++) __asm__("nop"); // min 120ns
    spi_xfer(SPI3,reg);
    usleep(35); //min 35us
    uint16_t result = spi_xfer(SPI3,0x00);
    for(i=0;i<12;i++) __asm__("nop"); // min 120ns
    gpio_set(GPIOA, GPIO15); // deassert FLOW chip select
    usleep(20); // min 20us

    return result;
}

// static void flow_write_register(uint8_t reg, uint8_t val)
// {
//     gpio_clear(GPIOA, GPIO15); // assert FLOW chip select
//     for(i=0;i<12;i++) __asm__("nop"); // min 120ns
//     spi_xfer(SPI3,1<<7|reg);
//     spi_xfer(SPI3,val);
//     usleep(35); // min 35us
//     gpio_set(GPIOA, GPIO15); // deassert FLOW chip select
//     usleep(100); // min 20us
// }

int main(void)
{
    // wait at least 40ms
    clock_init();
    timing_init();
    usleep(100000);
    spi_init();
    canbus_init();
    uavcan_init();
    uavcan_set_restart_cb(restart_request_handler);

    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6); // FLOW NRESET

    // set NRESET
    gpio_clear(GPIOA, GPIO6);
    // min 100ns
    usleep(2);
    gpio_set(GPIOA, GPIO6);

    usleep(1000);

//     flow_read_register(0x02);
//     flow_read_register(0x03);
//     flow_read_register(0x04);
//     flow_read_register(0x05);
//     flow_read_register(0x06);

    usleep(1000);

//     uint8_t prid = 0;
    uint16_t prid = flow_read_register(0x00);

    uint32_t last_print = 0;
    // main loop
    while(1) {
        uavcan_update();

        if (micros()-last_print > 10000) {
            uavcan_send_debug_key_value("PRID", prid);
            last_print = micros();
        }

        if (restart_req && (micros() - restart_req_us) > 1000) {
            // reset
            scb_reset_system();
        }
    }

    return 0;
}
