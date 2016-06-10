#include "esclink_messages.h"
#include "serial_esclink.h"
#include "serial.h"
#include "param.h"
#include "helpers.h"

#define FRAME_BUF_LEN 256

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

static char frame_buf[FRAME_BUF_LEN];
static uint16_t frame_len;

static void serial_esclink_parse_frame(void);
static int32_t serial_esclink_get_msg_size(enum ser_msg_type_t msgid);
static int32_t serial_esclink_encode_slip(char* buf_in, uint16_t len, char* buf_out, uint16_t max_len);
static bool serial_esclink_send_frame(char* buf, uint16_t len);

void serial_esclink_update(void)
{
    uint16_t i;
    volatile struct ringbuf_t* rxbuf = serial_get_rxbuf();
    uint16_t rxbuf_size = ringbuf_size(rxbuf);

    for(i=0; i<rxbuf_size; i++) {
        char byte;
        ringbuf_pop(rxbuf, &byte);

        if (byte == SLIP_END && frame_len > 0) {
            // we found a valid frame. parse it and discard.
            serial_esclink_parse_frame();
            frame_len = 0;
            break;
        } else if (frame_len >= FRAME_BUF_LEN) {
            // we filled the buffer without finding a frame. just discard the buffer.
            frame_len = 0;
            break;
        }

        // decode SLIP
        if (frame_len > 0 && byte == SLIP_ESC_END && frame_buf[frame_len-1] == SLIP_ESC) {
            frame_buf[frame_len-1] = SLIP_END;
        } else if (frame_len > 0 && byte == SLIP_ESC_ESC && frame_buf[frame_len-1] == SLIP_ESC) {
            frame_buf[frame_len-1] = SLIP_ESC;
        } else {
            frame_buf[frame_len++] = byte;
        }
    }

    struct ser_msg_param_index_invalid test_msg;
    test_msg.msg_id = MSG_TYPE_PARAM_INDEX_INVALID;
    test_msg.param_idx = 255;
    test_msg.crc16 = crc16_ccitt((char*)&test_msg, sizeof(test_msg)-2, 0);
    serial_esclink_send_frame((char*)&test_msg,sizeof(test_msg));
}


static void serial_esclink_parse_frame(void)
{
    uint16_t msg_id = frame_buf[0]<<8 | frame_buf[1];
    if (serial_esclink_get_msg_size(msg_id) != frame_len) {
        return;
    }
    uint16_t their_crc16 = frame_buf[frame_len-2]<<8 | frame_buf[frame_len-1];
    uint16_t our_crc16 = crc16_ccitt(frame_buf, frame_len-2, 0);
    if (their_crc16 != our_crc16) {
        return;
    }
    switch(msg_id) {
        case MSG_TYPE_PARAM_REQUEST_BY_INDEX: {
            struct ser_msg_param_request_by_index* msg = (struct ser_msg_param_request_by_index*)&frame_buf[0];

            if (param_index_in_range(msg->param_idx)) {
                struct ser_msg_param_value reply;
                reply.msg_id = MSG_TYPE_PARAM_VALUE;
                param_get_name_value_by_index(msg->param_idx, &reply.param_name[0], &reply.param_value);
                reply.crc16 = crc16_ccitt((char*)&reply, sizeof(reply)-2, 0);
                serial_esclink_send_frame((char*)&reply,sizeof(reply));
            } else {
                struct ser_msg_param_index_invalid reply;
                reply.msg_id = MSG_TYPE_PARAM_INDEX_INVALID;
                reply.param_idx = msg->param_idx;
                reply.crc16 = crc16_ccitt((char*)&reply, sizeof(reply)-2, 0);
                serial_esclink_send_frame((char*)&reply,sizeof(reply));
            }

            break;
        }
        case MSG_TYPE_PARAM_VALUE:
        default:
            break;
    }
}

static int32_t serial_esclink_get_msg_size(enum ser_msg_type_t msgid)
{
    switch(msgid) {
        case MSG_TYPE_PARAM_REQUEST_BY_INDEX:
            return sizeof(struct ser_msg_param_request_by_index);
        case MSG_TYPE_PARAM_VALUE:
            return sizeof(struct ser_msg_param_value);
        case MSG_TYPE_PARAM_INDEX_INVALID:
            return sizeof(struct ser_msg_param_index_invalid);
    }
    return -1;
}

static bool serial_esclink_send_frame(char* buf, uint16_t len)
{
    if (!serial_ready_to_send()) {
        return false;
    }
    int32_t encoded_len;
    encoded_len = serial_esclink_encode_slip(buf, len, serial_get_txbuf(), serial_get_txbuf_len());
    if (encoded_len == -1) {
        return false;
    }

    return serial_send_dma_preloaded(encoded_len);
}

static int32_t serial_esclink_encode_slip(char* buf_in, uint16_t len_in, char* buf_out, uint16_t max_len_out)
{
    uint16_t i;
    uint16_t len_out = 0;
    for (i=0; i<len_in; i++) {
        if (buf_in[i] == SLIP_END) {
            if (len_out > max_len_out-3) return -1;
            buf_out[len_out++] = SLIP_ESC;
            buf_out[len_out++] = SLIP_ESC_END;
        } else if (buf_in[i] == SLIP_ESC) {
            if (len_out > max_len_out-3) return -1;
            buf_out[len_out++] = SLIP_ESC;
            buf_out[len_out++] = SLIP_ESC_ESC;
        } else {
            if (len_out > max_len_out-2) return -1;
            buf_out[len_out++] = buf_in[i];
        }
    }
    buf_out[len_out++] = SLIP_END;
    return len_out;
}
