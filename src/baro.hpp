#pragma once
#include <cstdint>

void baro_setup();

// Can return NaN if data hasn't been recieved from the barometer yet.
float baro_get_altitude();
int32_t baro_get_temp();

void baro_print();
