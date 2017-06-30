#pragma once

#ifndef BL_PACKED
#define BL_PACKED __attribute__((packed))
#endif

enum bl_msg_id_t {
    BL_MSGID_UNKNOWN=0,
    BL_MSGID_BOOT=1,
    BL_MSGID_FLASH_FROM_UAVCAN=2,
    BL_MSGID_APP_INIT=3
};

static struct uavcan_node_info {
    uint8_t canbus_id;
    uint8_t node_id;
    uint32_t bitrate;
} BL_PACKED;

static struct bl_msg_boot {
    uint8_t flags_to_pass;
} BL_PACKED;

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
