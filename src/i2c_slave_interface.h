#pragma once

#include <stdint.h>

void i2c_slave_init(void);
uint32_t i2c_slave_retrieve_led_color_hex(void);
