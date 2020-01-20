#include <Arduino.h>

#include "radio.hpp"
#include "util.hpp"
#include "scheduler.hpp"

void radio_sync()
{
	XBEE_SERIAL.write("\xdb\x69\xc0\x78", 4);
}

void radio_send(const Packet &pkt)
{
	const byte *pkt_buf = reinterpret_cast<const byte *>(&pkt);
	XBEE_SERIAL.write(pkt_buf, pkt.getLen());
	uint8_t ck = 0;
	for (size_t i = 0; i < pkt.getLen(); ++i) {
		ck ^= pkt_buf[i];
	}
	XBEE_SERIAL.write(ck);
}

void radio_setup()
{
	XBEE_SERIAL.begin(115200);
	scheduler_add(TaskId::PacketSync, Task(radio_sync, 20 * KALMAN_PERIOD * 1000L));
}
