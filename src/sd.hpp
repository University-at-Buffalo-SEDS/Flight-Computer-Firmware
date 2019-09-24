#pragma once
#include <stdint.h>
#include <Arduino.h>

#include "kalman.hpp"

struct SdDataRecord {
	uint32_t time_ms;
	KalmanState state;
	int32_t temp;
	float altitude, accel_x, accel_y, accel_z, lat, lon, gps_alt;
	uint16_t batt_v;
	//struct {
	//	uint16_t year;
	//	uint8_t month, day, hour, minute, second;
	//	uint16_t millisecond;
	//} gps_time;
};


void sd_setup();
void sd_log(const SdDataRecord &data);
void sd_commit(bool enable);
Print *sd_messages();
