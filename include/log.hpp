#pragma once
#include <cstdint>
#include "kalman.hpp"

struct LogMessage {
	uint32_t time_ms;
	KalmanState state;
	float altitude, accel_x, accel_y, accel_z, lat, lon, gps_alt;
	int16_t temp;
	uint16_t batt_v;
	//struct {
	//	uint16_t year;
	//	uint8_t month, day, hour, minute, second;
	//	uint16_t millisecond;
	//} gps_time;
};

void log_setup();
void log_start();
void log_stop();
void log_add(const LogMessage &data);
void log_print();
