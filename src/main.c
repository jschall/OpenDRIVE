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
#include "helpers.h"
// #include <stdio.h>

struct jump_info_s {
    uint32_t stacktop;
    uint32_t entrypoint;
    uint32_t crc32;
};

static bool restart_req = false;
static uint32_t restart_req_us = 0;

static bool restart_request_handler(void)
{
    restart_req = true;
    restart_req_us = micros();
    return true;
}

int main(void)
{
    clock_init();
    timing_init();
    canbus_init();
    uavcan_init();
    uavcan_set_restart_cb(restart_request_handler);

    // main loop
    while(1) {
        uavcan_update();

        if (restart_req && (micros() - restart_req_us) > 1000) {
            scb_reset_system();
        }
    }

    return 0;
}
