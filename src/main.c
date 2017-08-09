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
#include <math.h>
#include "icm.h"
#include "dycoled.h"
#include "i2c_slave_interface.h"


#define CANBUS_AUTOBAUD_SWITCH_INTERVAL_US 1000000
#define CANBUS_AUTOBAUD_TIMEOUT_US 10000000

static volatile const struct shared_app_parameters_s shared_app_parameters __attribute__((section(".app_descriptor"),used)) = {
    .boot_delay_sec = APP_CONFIG_BOOT_DELAY_SEC,
    .canbus_disable_auto_baud = !APP_CONFIG_CAN_AUTO_BAUD_ENABLE,
    .canbus_baudrate = APP_CONFIG_CAN_DEFAULT_BAUDRATE,
    .canbus_local_node_id = APP_CONFIG_CAN_LOCAL_NODE_ID
};

static volatile const struct shared_app_descriptor_s shared_app_descriptor __attribute__((section(".app_descriptor"),used)) = {
    .signature = SHARED_APP_DESCRIPTOR_SIGNATURE,
    .image_crc = 0,
    .image_size = 0,
    .vcs_commit = GIT_HASH,
    .major_version = 1,
    .minor_version = 0,
    .parameters_fmt = SHARED_APP_PARAMETERS_FMT,
    .parameters_ignore_crc64 = true,
    .parameters = {&shared_app_parameters, 0}
};

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

static void set_uavcan_node_info(void) {
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


static void restart_request_handler(struct uavcan_transfer_info_s transfer_info, uint64_t magic) {
    if (magic == 0xACCE551B1E) {
        uavcan_send_restart_response(&transfer_info, true);
        uint32_t tbegin_us = micros();
        while (micros()-tbegin_us < 1000) {
            uavcan_update();
        }

        union shared_msg_payload_u msg;
        fill_shared_canbus_info(&msg.canbus_info);
        shared_msg_finalize_and_write(SHARED_MSG_CANBUS_INFO, &msg);

        scb_reset_system();
    } else {
        uavcan_send_restart_response(&transfer_info, false);
    }
}

static void file_beginfirmwareupdate_handler(struct uavcan_transfer_info_s transfer_info, uint8_t source_node_id, const char* path) {
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

static void uavcan_ready_handler(void) {
    icm_init();
}

static bool canbus_autobaud_running;
static struct canbus_autobaud_state_s autobaud_state;
static void on_canbus_baudrate_confirmed(uint32_t canbus_baud);

static void begin_canbus_autobaud(void) {
    uint32_t canbus_baud;
    if (shared_msg_valid && canbus_baudrate_valid(shared_msg.canbus_info.baudrate)) {
        canbus_baud = shared_msg.canbus_info.baudrate;
    } else if (canbus_baudrate_valid(shared_app_parameters.canbus_baudrate)) {
        canbus_baud = shared_app_parameters.canbus_baudrate;
    } else {
        canbus_baud = 1000000;
    }

    bool canbus_autobaud_enable;
    if (shared_msg_valid && canbus_baudrate_valid(shared_msg.canbus_info.baudrate)) {
        canbus_autobaud_enable = false;
    } else if (shared_app_parameters.canbus_disable_auto_baud) {
        canbus_autobaud_enable = false;
    } else {
        canbus_autobaud_enable = true;
    }

    if (canbus_autobaud_enable) {
        canbus_autobaud_start(&autobaud_state, canbus_baud, CANBUS_AUTOBAUD_SWITCH_INTERVAL_US);
        canbus_autobaud_running = true;
    } else {
        on_canbus_baudrate_confirmed(canbus_baud);
    }
}

static void update_canbus_autobaud(void) {
    if (!canbus_autobaud_running) {
        return;
    }

    uint32_t canbus_baud = canbus_autobaud_update(&autobaud_state);
    if (autobaud_state.success) {
        on_canbus_baudrate_confirmed(canbus_baud);
        canbus_autobaud_running = false;
    }

}

static void on_canbus_baudrate_confirmed(uint32_t canbus_baud) {
    canbus_init(canbus_baud, false);
    uavcan_init();

    set_uavcan_node_info();
    uavcan_set_uavcan_ready_cb(uavcan_ready_handler);
    uavcan_set_restart_cb(restart_request_handler);
    uavcan_set_file_beginfirmwareupdate_cb(file_beginfirmwareupdate_handler);
    uavcan_set_node_mode(UAVCAN_MODE_OPERATIONAL);

    if (shared_msg_valid && shared_msg.canbus_info.local_node_id > 0 && shared_msg.canbus_info.local_node_id <= 127) {
        uavcan_set_node_id(shared_msg.canbus_info.local_node_id);
    } else if (shared_app_parameters.canbus_local_node_id > 0 && shared_app_parameters.canbus_local_node_id <= 127) {
        uavcan_set_node_id(shared_msg.canbus_info.local_node_id);
    }
}

int main(void) {
    init_clock();
    timing_init();

    shared_msg_valid = shared_msg_check_and_retreive(&shared_msgid, &shared_msg);
    shared_msg_clear();

    begin_canbus_autobaud();

    i2c_slave_init();

    // main loop
    while(1) {
        update_canbus_autobaud();
        uavcan_update();

        icm_update();
        dycoled_set_color_all_hex(i2c_slave_retrieve_led_color_hex());
        dycoled_update();
    }

    return 0;
}
