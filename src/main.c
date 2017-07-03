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
#include <string.h>
#include <libopencm3/stm32/flash.h>

extern unsigned _app_begin, _app_end, _app_bl_shared_begin, _app_bl_shared_end;

#define FLASH_PAGE_SIZE 1024
#define APP_SECTION_SIZE ((uint32_t)(&_app_end - &_app_begin))
#define APP_SECTION_PAGES (APP_SECTION_SIZE/FLASH_PAGE_SIZE)
#define APP_BL_SHARED_SIZE ((uint32_t)(&_app_bl_shared_end - &_app_bl_shared_begin))

static uint8_t* app_bl_shared = (uint8_t*)&_app_bl_shared_begin;
static uint8_t* app_flash_section = (uint8_t*)&_app_begin;// __attribute__((section(".app")));

static bool restart_req = false;
static uint32_t restart_req_us;

static bool restart_request_handler(void)
{
    restart_req = true;
    restart_req_us = micros();
    return true;
}

static void do_jump(uint32_t stacktop, uint32_t entrypoint)
{
    asm volatile(
        "msr msp, %0	\n"
        "bx	%1	\n"
        : : "r"(stacktop), "r"(entrypoint) :);

    // just to keep noreturn happy
    for (;;) ;
}

static void do_boot(void);

static struct {
    uint32_t ofs;
    uint8_t transfer_id;
    uint32_t last_req_ms;
    uint8_t source_node_id;
    char path[201];
} flash_state;

static bool __attribute__ ((noinline)) flash_program_half_word(uint16_t* addr, uint16_t* src)
{
    bool ret;
    // 1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
    flash_wait_for_last_operation();
    // 2. Set the PG bit in the FLASH_CR register.
    FLASH_CR |= FLASH_CR_PG;
    // 3. Perform the data write (half-word) at the desired address.
    *addr = *src;
    // 4. Wait until the BSY bit is reset in the FLASH_SR register.
    flash_wait_for_last_operation();
    // 5. Check the EOP flag in the FLASH_SR register (it is set when the programming operation has succeded), and then clear it by software
    ret = (FLASH_SR & FLASH_SR_EOP) != 0 && (FLASH_SR & (1UL<<2)) == 0 && (FLASH_SR & (1UL<<4)) == 0;
    FLASH_SR &= ~FLASH_SR_EOP;
    // also clear PG for good measure
    FLASH_CR &= ~FLASH_CR_PG;

    flash_wait_for_last_operation();

    return ret;
}

static bool __attribute__ ((noinline)) flash_erase_page(void* addr)
{
    bool ret;
    // 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register.
    flash_wait_for_last_operation();
    // 2. Set the PER bit in the FLASH_CR register
    FLASH_CR |= FLASH_CR_PER;
    // 3. Program the FLASH_AR register to select a page to erase
    FLASH_AR = (uint32_t)&addr;
    // 4. Set the STRT bit in the FLASH_CR register (see below note)
    FLASH_CR |= FLASH_CR_STRT;
    // 5. Wait for the BSY bit to be reset
    flash_wait_for_last_operation();
    // 6. Check the EOP flag in the FLASH_SR register (it is set when the erase operation has succeded), and then clear it by software.
    ret = (FLASH_SR & FLASH_SR_EOP) != 0;
    FLASH_SR &= ~FLASH_SR_EOP;
    // also clear PER for good measure
    FLASH_CR &= ~FLASH_CR_PER;

    return ret;
}

static void write_data_to_flash(const uint8_t* data, uint16_t data_len)
{
    for (uint16_t i=0; i<data_len; i+=2) {
        flash_program_half_word(&((uint16_t*)app_flash_section)[i], &((uint16_t*)data)[i]);
    }
}

static void begin_flash_from_path(uint8_t source_node_id, const char* path)
{
    memset(&flash_state, 0, sizeof(flash_state));
    flash_state.ofs = 0;
    flash_state.source_node_id = source_node_id;
    strncpy(flash_state.path, path, 200);
    flash_state.transfer_id = uavcan_send_file_read_request(flash_state.source_node_id, flash_state.ofs, flash_state.path);
    flash_state.last_req_ms = millis();

    for (uint32_t i=0; i<APP_SECTION_SIZE; i+=FLASH_PAGE_SIZE) {
        flash_erase_page(&app_flash_section[i]);
    }
}

static void file_read_response_handler(uint8_t transfer_id, int16_t error, const uint8_t* data, uint16_t data_len, bool eof)
{
    if (transfer_id == flash_state.transfer_id && error == 0) {
        write_data_to_flash(data, data_len);

        if (eof) {
            do_boot();
        } else {
            flash_state.ofs += data_len;
            flash_state.transfer_id = uavcan_send_file_read_request(flash_state.source_node_id, flash_state.ofs, flash_state.path);
            flash_state.last_req_ms = millis();
        }
    }
}

static void file_beginfirmwareupdate_handler(struct uavcan_transfer_info_s transfer_info, uint8_t source_node_id, const char* path)
{
    uavcan_send_file_beginfirmwareupdate_response(&transfer_info, 0, "");
    begin_flash_from_path(source_node_id, path);
}

#ifndef SHARED_MSG_PACKED
#define SHARED_MSG_PACKED __attribute__((packed))
#endif

#define BOOT_MAGIC 0xDEADBEEF

enum shared_msg_t {
    SHARED_MSG_UNKNOWN = 0,
    SHARED_MSG_BOOT = 1,
};

struct shared_msg_boot_s {
    uint8_t msgid;
    uint8_t len;
    uint32_t magic;
    uint32_t crc32;
} SHARED_MSG_PACKED;

static uint8_t shared_msg_length(uint8_t msgid) {
    switch((enum shared_msg_t)msgid) {
        case SHARED_MSG_UNKNOWN:
            break;
        case SHARED_MSG_BOOT:
            return sizeof(struct shared_msg_boot_s);
    };

    return 0;
}

static void do_boot(void) {
    struct shared_msg_boot_s* msg = (struct shared_msg_boot_s*)&app_bl_shared;
    msg->msgid = SHARED_MSG_BOOT;
    msg->magic = BOOT_MAGIC;
    msg->crc32 = crc32((uint8_t*)&msg, sizeof(msg)-sizeof(uint32_t), 0);

    scb_reset_system();
}

int main(void)
{
    enum shared_msg_t shared_msg_id = (enum shared_msg_t)app_bl_shared[0];
    uint8_t shared_msg_len = app_bl_shared[1];

    bool shared_msg_valid = false;

    if (shared_msg_len >= 6 && shared_msg_len == shared_msg_length(shared_msg_id)) {
        uint32_t* shared_msg_crc32_provided = (uint32_t*)&app_bl_shared[shared_msg_len-sizeof(uint32_t)];
        uint32_t shared_msg_crc32_computed = crc32(app_bl_shared, shared_msg_len-sizeof(uint32_t), 0);

        if (*shared_msg_crc32_provided == shared_msg_crc32_computed) {
            shared_msg_valid = true;
        }
    }

    // read the app/bl shared memory, look for the command to boot immediately
    if (shared_msg_valid && shared_msg_id == SHARED_MSG_BOOT) {
        struct shared_msg_boot_s* cmd = (struct shared_msg_boot_s*)&app_bl_shared;
        if (cmd->magic == BOOT_MAGIC) {
            memset(&app_bl_shared,0,APP_BL_SHARED_SIZE);
            do_jump(((uint32_t*)app_flash_section)[0], ((uint32_t*)app_flash_section)[1]);
        }
    }

    clock_init();
    timing_init();
    canbus_init();
    uavcan_init();
    uavcan_set_restart_cb(restart_request_handler);
    uavcan_set_file_beginfirmwareupdate_cb(file_beginfirmwareupdate_handler);
    uavcan_set_file_read_response_cb(file_read_response_handler);

    // main loop
    while(1) {
        uavcan_update();

        if (restart_req && (micros() - restart_req_us) > 1000) {
            do_boot();
        }
    }

    return 0;
}
