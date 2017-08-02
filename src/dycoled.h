#pragma once

#include "dycoled_gen.h"
#include <stdint.h>

#ifndef NUM_DYCOLEDS
#define NUM_DYCOLEDS 4
#endif


void dycoled_set_color_all_rgb(uint8_t r, uint8_t g, uint8_t b);
void dycoled_set_color_rgb(uint32_t idx, uint8_t r, uint8_t g, uint8_t b);
void dycoled_set_color_all_hex(uint32_t color_hex);
void dycoled_set_color_hex(uint32_t idx, uint32_t color_hex);

void dycoled_update(void);
