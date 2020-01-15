#pragma once
#include "util.hpp"

struct __attribute__((__packed__)) Packet {
	// Should be sorted in order of size in order to minimize space wasted for alignment.
	// The start of the sync sequence should also be an invalid value for the first value listed.
	// Size should also be kept below 64 bytes to avoid delays (STM32duino SERIAL_TX_BUFFER_SIZE).
	uint32_t millis;
	float alt, vel, acc, raw_alt, raw_acc, lat, lon, apogee;
	uint16_t temp, batt_v;
	FlightPhase phase;

	Packet(FlightPhase phase, uint32_t millis, float alt, float vel, float acc,
			float raw_alt, float raw_acc, float lat, float lon, float apogee,
			int16_t temp, uint16_t batt_v) :
		millis(millis), alt(alt), vel(vel), acc(acc),
		raw_alt(raw_alt), raw_acc(raw_acc), lat(lat), lon(lon), apogee(apogee),
		temp(temp), batt_v(batt_v), phase(phase)
	{}

	int getLen() const { return sizeof(Packet); }
};

static_assert(sizeof(Packet) == 41, "Packet size changed.");

void radio_setup();
void radio_send(const Packet &p);
