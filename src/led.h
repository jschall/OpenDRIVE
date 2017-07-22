#pragma once

#include <stdint.h>

typedef void (*led_write_byte_func_t)(uint8_t byte);

struct led_color_s {
    uint8_t bytes[3];
};

struct led_instance_s {
    uint32_t num_leds;
    struct led_color_s* led_colors;
    uint8_t num_leading_zeros;
    uint32_t output_stream_length;
    led_write_byte_func_t write_byte;
};

void led_make_brg_color_rgb(uint8_t r, uint8_t g, uint8_t b, struct led_color_s* ret);
void led_make_brg_color_hex(uint32_t color, struct led_color_s* ret);
uint32_t led_get_buf_size(uint32_t num_leds);
void led_write(uint32_t num_leds, struct led_color_s* led_colors, led_write_byte_func_t write_byte);
uint32_t led_write_buf(uint32_t num_leds, struct led_color_s* led_colors, uint8_t* buf, uint32_t buf_size);
