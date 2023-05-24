#pragma once
#include <cstdint>

void baro_setup();

// Can return NaN if data hasn't been received from the barometer yet.
float baro_get_altitude();
int16_t baro_get_temp();
float baro_get_pressure();

void baro_print();
