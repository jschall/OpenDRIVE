#include "dycoled_gen.h"
#include <stdint.h>

typedef void (*dycoled_gen_write_byte_func_internal_t)(uint32_t index, uint8_t byte, void* context);

void dycoled_gen_make_brg_color_rgb(uint8_t r, uint8_t g, uint8_t b, struct dycoled_gen_color_s* ret) {
    ret->bytes[0] = b;
    ret->bytes[1] = r;
    ret->bytes[2] = g;
}

void dycoled_gen_make_brg_color_hex(uint32_t color, struct dycoled_gen_color_s* ret) {
    dycoled_gen_make_brg_color_rgb((uint8_t)(color>>16), (uint8_t)(color>>8), (uint8_t)color, ret);
}

uint32_t dycoled_gen_get_buf_size(uint32_t num_leds) {
    uint32_t min_bits = num_leds*25+50;
    return (min_bits+7)/8;
}

static uint32_t _dycoled_gen_write(uint32_t num_leds, struct dycoled_gen_color_s* dycoled_gen_colors, dycoled_gen_write_byte_func_internal_t write_byte, void* context) {
    const uint32_t min_bits = num_leds*25+50;
    const uint8_t num_leading_zeros = 8-min_bits%8 + 50;
    const uint8_t header_bit_length = num_leading_zeros+1;
    const uint8_t header_min_byte_length = (header_bit_length+7)/8;
    const uint32_t output_stream_length = (min_bits+7)/8;

    uint8_t* dycoled_gen_color_byte_array = (uint8_t*)dycoled_gen_colors;

    uint32_t output_idx;
    for (output_idx = 0; output_idx < (uint8_t)(header_min_byte_length-1); output_idx++) {
        write_byte(output_idx, 0, context);
    }

    uint16_t out_byte;
    out_byte = ((uint16_t)dycoled_gen_color_byte_array[0]>>1)|0x80U;
    out_byte >>= num_leading_zeros%8;
    write_byte(output_idx, out_byte&0xFFU, context);
    output_idx++;

    // bit index of the next bit to be written
    uint32_t dycoled_gen_data_bit_offset = 7 - num_leading_zeros%8;

    for (; output_idx < output_stream_length-1; output_idx++) {
        uint32_t dycoled_gen_first_byte_idx = dycoled_gen_data_bit_offset/8;
        uint32_t dycoled_gen_second_byte_idx = dycoled_gen_first_byte_idx+1;
        uint8_t bits_from_first_byte = 8-dycoled_gen_data_bit_offset%8;

        out_byte = ((uint16_t)dycoled_gen_color_byte_array[dycoled_gen_first_byte_idx])<<8;

        if (dycoled_gen_second_byte_idx%3 == 0) {
            // byte contains delimiter
            out_byte |= (dycoled_gen_color_byte_array[dycoled_gen_second_byte_idx] >> 1) | 0x80U;
            dycoled_gen_data_bit_offset += 7;
        } else {
            out_byte |= dycoled_gen_color_byte_array[dycoled_gen_second_byte_idx];
            dycoled_gen_data_bit_offset += 8;
        }

        out_byte >>= bits_from_first_byte;

        write_byte(output_idx, out_byte&0xFFU, context);
    }

    write_byte(output_idx, dycoled_gen_color_byte_array[dycoled_gen_data_bit_offset/8], context);
    output_idx++;

    return output_idx;
}

static void _dycoled_gen_call_write_byte_cb(uint32_t index, uint8_t byte, void* context) {
    (void)index;
    ((dycoled_gen_write_byte_func_t)context)(byte);
}

uint32_t dycoled_gen_write(uint32_t num_leds, struct dycoled_gen_color_s* dycoled_gen_colors, dycoled_gen_write_byte_func_t write_byte) {
    return _dycoled_gen_write(num_leds, dycoled_gen_colors, _dycoled_gen_call_write_byte_cb, (void*)write_byte);
}

static void _dycoled_gen_write_buf_cb(uint32_t index, uint8_t byte, void* context) {
    uint8_t* buf = context;
    buf[index] = byte;
}

uint32_t dycoled_gen_write_buf(uint32_t num_leds, struct dycoled_gen_color_s* dycoled_gen_colors, uint8_t* buf, uint32_t buf_size) {
    if (buf_size < dycoled_gen_get_buf_size(num_leds)) {
        return 0;
    }

    return _dycoled_gen_write(num_leds, dycoled_gen_colors, _dycoled_gen_write_buf_cb, (void*)buf);
}
