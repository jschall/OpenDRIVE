#pragma once

#include <stdbool.h>

struct ak09916_measurement_s {
    float x,y,z;
    bool hofl;
};

void icm_init(void);
void icm_update(void);
