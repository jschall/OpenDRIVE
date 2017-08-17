#pragma once

#include <stdint.h>

typedef void (*dycoled_gen_write_byte_func_t)(uint8_t byte);

struct dycoled_gen_color_s {
    uint8_t bytes[3];
};

void dycoled_gen_make_brg_color_rgb(uint8_t r, uint8_t g, uint8_t b, struct dycoled_gen_color_s* ret);
void dycoled_gen_make_brg_color_hex(uint32_t color, struct dycoled_gen_color_s* ret);
uint32_t dycoled_gen_get_buf_size(uint32_t num_leds);
uint32_t dycoled_gen_write(uint32_t num_leds, struct dycoled_gen_color_s* dycoled_gen_colors, dycoled_gen_write_byte_func_t write_byte);
uint32_t dycoled_gen_write_buf(uint32_t num_leds, struct dycoled_gen_color_s* dycoled_gen_colors, uint8_t* buf, uint32_t buf_size);
