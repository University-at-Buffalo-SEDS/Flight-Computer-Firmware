#pragma once
#include "kalman.hpp"
#include "util.hpp"

#include <cstddef>
#include <cstdint>

struct LogMessage {
	uint32_t time_ms;
	KalmanState state;
	float altitude, accel_x, accel_y, accel_z, lat, lon, gps_alt;
	int16_t temp;
	uint16_t batt_v, sys_v;
	uint8_t checksum;

	LogMessage() = default;

	LogMessage(uint32_t time_ms, KalmanState &state,
		float altitude, float accel_x, float accel_y, float accel_z,
		float lat, float lon, float gps_alt,
		int16_t temp, uint16_t batt_v, uint16_t sys_v) :
		time_ms(time_ms), state(state),
		altitude(altitude), accel_x(accel_x), accel_y(accel_y), accel_z(accel_z),
		lat(lat), lon(lon), gps_alt(gps_alt),
		temp(temp), batt_v(batt_v), sys_v(sys_v), checksum(0)
	{
		checksum = struct_checksum(*this);
	}
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
void log_print_all();
