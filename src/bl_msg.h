#pragma once

#ifndef BL_PACKED
#define BL_PACKED __attribute__((packed))
#endif

enum bl_msg_id_t {
    BL_MSGID_FLASH_FROM_UAVCAN=0,
    BL_MSGID_APP_INIT=1
};

static struct bl_msg_flash_from_uavcan {
    uint8_t canbus_id;
    uint8_t node_id;
    uint32_t bitrate;
    uint8_t source_node;
    char path[201];
} BL_PACKED;

static struct bl_msg_app_info {
    uint8_t canbus_id;
    uint8_t node_id;
    uint32_t bitrate;
    uint8_t coa[255];
} BL_PACKED;
