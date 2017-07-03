#pragma once

#include <stdint.h>
#include <stdbool.h>

enum uavcan_loglevel_t {
    UAVCAN_LOGLEVEL_DEBUG = 0,
    UAVCAN_LOGLEVEL_INFO = 1,
    UAVCAN_LOGLEVEL_WARNING = 2,
    UAVCAN_LOGLEVEL_ERROR = 3
};

struct uavcan_transfer_info_s {
    void* canardInstance;
    uint8_t remote_node_id;
    uint8_t transfer_id;
    uint8_t priority;
};

typedef bool (*restart_handler_ptr)(void);
typedef void (*esc_rawcommand_handler_ptr)(uint8_t len, int16_t* commands);
typedef void (*file_beginfirmwareupdate_handler_ptr)(struct uavcan_transfer_info_s transfer_info, uint8_t source_node_id, const char* path);
typedef void (*file_read_response_handler_ptr)(uint8_t transfer_id, int16_t error, const uint8_t* data, uint16_t data_len, bool eof);

void uavcan_init(void);
void uavcan_update(void);
void uavcan_set_restart_cb(restart_handler_ptr cb);
void uavcan_set_esc_rawcommand_cb(esc_rawcommand_handler_ptr cb);
void uavcan_set_file_beginfirmwareupdate_cb(file_beginfirmwareupdate_handler_ptr cb);
void uavcan_set_file_read_response_cb(file_read_response_handler_ptr cb);

void uavcan_send_debug_key_value(const char* name, float val);
void uavcan_send_debug_logmessage(enum uavcan_loglevel_t log_level, const char* source, const char* text);
void uavcan_send_file_beginfirmwareupdate_response(struct uavcan_transfer_info_s* transfer_info, uint8_t error, const char* error_message);
uint8_t uavcan_send_file_read_request(uint8_t remote_node_id, const uint64_t offset, const char* path);
