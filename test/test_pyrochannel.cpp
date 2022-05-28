#include "config.hpp"


void setup() {

    // Sets both pins for each pyrochannel to LOW
	for (const ChannelConfig &c : channel_config) {
		pinMode(c.fire_pin, OUTPUT);
		digitalWrite(c.fire_pin, LOW);
	}
    Serial.println("Channel pins set to low. Waiting to fire.");
    delay(TEST_FIRE_DELAY);


    for (const ChannelConfig &c : channel_config) {
		digitalWrite(c.fire_pin, HIGH);
	}
    Serial.println("Fired");
    delay(TEST_FIRE_UPTIME);



    for (const ChannelConfig &c : channel_config) {
		digitalWrite(c.fire_pin, LOW);
	}
    Serial.println("Test complete. Pins set to low.");
}


void loop() {


}