#include <unity.h>
#include <cstring>
#include <cstdio>

#include "config.hpp"


void test_pyrochannels() {

    // Sets both pins for each pyrochannel to LOW
	for (const ChannelConfig &c : channel_config) {
		pinMode(c.fire_pin, OUTPUT);
		digitalWrite(c.fire_pin, LOW);
	}

    Serial.println("Channel pins set to low. Waiting to fire.");
    // delay(TEST_FIRE_DELAY);

	for (size_t i = 0; i < 2; ++i) {
		const ChannelConfig &config = channel_config[i];

		Serial.println("Channel pins set to low. Waiting to fire.");
   		delay(TEST_FIRE_DELAY);

		digitalWrite(config.fire_pin, HIGH);

		Serial.print("Fired ");
		Serial.println(config.fire_pin);
    	delay(TEST_FIRE_UPTIME);

		digitalWrite(config.fire_pin, LOW);
	}

    Serial.println("Test complete. Pins set to low.");
}

int main() {
	UNITY_BEGIN();

	RUN_TEST(test_pyrochannels);

	UNITY_END();
	return 0;
}