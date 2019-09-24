#pragma once
#include "util.hpp"

struct __attribute__((__packed__)) Packet {
	// Should be sorted in order of size in order to minimize space wasted for alignment.
	// The start of the sync sequence should also be an invalid value for the first value listed.
	// Size should also be kept below 40 bytes to avoid delays (the Teensy Serial stdlib buffers up to 40 bytes on Serial2).
	// Note: Must update getLen on every change!
	int32_t temp;
	uint32_t millis;
	float alt, vel, acc, raw_alt, raw_acc, lat, lon;
	uint16_t batt_v;
	FlightPhase phase;

	Packet(FlightPhase phase, uint32_t millis, float alt, float vel, float acc,
			float raw_alt, float raw_acc, float lat, float lon,
			int32_t temp, uint16_t batt_v) :
		temp(temp), millis(millis), alt(alt), vel(vel), acc(acc),
		raw_alt(raw_alt), raw_acc(raw_acc), lat(lat), lon(lon),
		batt_v(batt_v), phase(phase)
	{
		if (this->temp > 100000) {
			this->temp = 100000;
		}
	}

	int getLen() const { return sizeof(Packet); }
};

static_assert(sizeof(Packet) == 39, "Packet size changed.");

void radio_setup();
void radio_send(const Packet &p);
