#pragma once

#include <stdint.h>
#include <stdbool.h>

void i2c_slave_init(void);
uint32_t i2c_slave_retrieve_led_color_hex(void);
void i2c_slave_new_compass_data(float x, float y, float z, bool hofl);
