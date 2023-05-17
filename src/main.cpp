#include <Arduino.h>
#include <Wire.h>

#if defined (USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

void setup()
{

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

#if defined (USBCON) && defined(USBD_USE_CDC)
	usb_serial.begin();
#else
	Serial.begin(9'600);
#endif
	Serial3.begin(57600);

	Serial.println(F("Flight Computer " __DATE__ " " __TIME__));
	Serial3.println(F("Flight Computer " __DATE__ " " __TIME__));
}

void loop()
{
	uint32_t data = 0;
	if (Serial.available() > 0) {
		data = Serial.read();
		Serial.write(data);
		Serial3.write(data);
	}
	if (Serial3.available() > 0) {
		data = Serial3.read();
		Serial.write(data);
		Serial3.write(data);
	}
	delay(10);
}

 