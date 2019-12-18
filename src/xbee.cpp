#include <Arduino.h>

#include "radio.hpp"
#include "util.hpp"
#include "scheduler.hpp"

#define xbee_serial Serial2

void radio_sync()
{
	xbee_serial.write("\xdb\x69\xc0\x78", 4);
}

void radio_send(const Packet &pkt)
{
	const byte *pkt_buf = reinterpret_cast<const byte *>(&pkt);
	xbee_serial.write(pkt_buf, pkt.getLen());
	uint8_t ck = 0;
	for (uint8_t i = 0; i < pkt.getLen(); ++i) {
		ck ^= pkt_buf[i];
	}
	xbee_serial.write(ck);
}

void radio_setup()
{
	xbee_serial.begin(115200);
	scheduler_add(TaskId::PacketSync, Task(radio_sync, 20 * KALMAN_PERIOD * 1000L));
}
