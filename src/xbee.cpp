// #include "radio.hpp"
// #include "scheduler.hpp"
// #include "util.hpp"

// #include <Arduino.h>

// void radio_sync()
// {
// 	XBEE_SERIAL.write("\xdb\x69\xc0\x78", 4);
// }

// void radio_send(const Packet &pkt)
// {
// 	const byte *pkt_buf = reinterpret_cast<const byte *>(&pkt);
// 	XBEE_SERIAL.write(pkt_buf, pkt.getLen());
// }

// void radio_setup()
// {
// 	XBEE_SERIAL.begin(115200);
// 	scheduler_add(TaskId::PacketSync, Task(radio_sync, 20 * KALMAN_PERIOD * 1000L, 20));
// }
