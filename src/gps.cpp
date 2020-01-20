#include <Adafruit_GPS.h>
#include "gps.hpp"
#include "scheduler.hpp"
#include "util.hpp"

void gps_step();

static Adafruit_GPS gps(&GPS_SERIAL);

static void gps_read()
{
	while (GPS_SERIAL.available()) {
		gps.read();
	}
}

void gps_setup()
{
	GPS_SERIAL.begin(9600);
	delay(10);
	// We only need RMC and GGA commands
	GPS_SERIAL.println(F(PMTK_SET_NMEA_OUTPUT_RMCGGA));
#if GPS_UPDATE_RATE == 1
#define GPS_BAUD 9600
	GPS_SERIAL.println(F(PMTK_SET_NMEA_UPDATE_1HZ));
	GPS_SERIAL.println(F(PMTK_API_SET_FIX_CTL_1HZ));
#elif GPS_UPDATE_RATE == 2
#define GPS_BAUD 9600
	GPS_SERIAL.println(F(PMTK_SET_NMEA_UPDATE_2HZ));
	GPS_SERIAL.println(F(PMTK_API_SET_FIX_CTL_5HZ));
#elif GPS_UPDATE_RATE == 5
#define GPS_BAUD 9600
	GPS_SERIAL.println(F(PMTK_SET_NMEA_UPDATE_5HZ));
	GPS_SERIAL.println(F(PMTK_API_SET_FIX_CTL_5HZ));
#elif GPS_UPDATE_RATE == 10
#define GPS_BAUD 57600
	// Can go up to 10Hz, but only up to 5Hz for actual fixes.
	GPS_SERIAL.println(F(PMTK_SET_NMEA_UPDATE_10HZ));
	GPS_SERIAL.println(F(PMTK_API_SET_FIX_CTL_5HZ));
#warning "GPS update rate is 10Hz, but fixes will only be obtained at 5Hz."
#else
#error "GPS Update speed must be 1, 2, 5, or 10Hz"
#endif

#if GPS_BAUD == 9600
#elif GPS_BAUD == 57600
	// Need to set this if you use 10Hz updates or enable
	// more than RMC+GGA at a high update speed.
	GPS_SERIAL.println(F(PMTK_SET_BAUD_57600));
	GPS_SERIAL.begin(57600);
#else
#error "GPS_BAUD can only be 9600 or 57600."
#endif

	// DGPS: Accuracy improvement measures.
	GPS_SERIAL.println(F(PMTK_ENABLE_SBAS));
	GPS_SERIAL.println(F(PMTK_ENABLE_WAAS));

	delay(1000);

	// Ask for the release and version
	GPS_SERIAL.println(F(PMTK_Q_RELEASE));

	// This should be fast enough to parse 25-30 NMEA sentences per second.
	scheduler_add(TaskId::Gps, Task(gps_step, 32'000, 100, 40'000));
	// Each byte takes about 10 bits (1 start bit + 8 data bits + 1 stop bit).
	// Additionally, Teensy maintains a 64 byte buffer by default.
	// Subtract 100 so that we're never late.
	scheduler_add(TaskId::GpsRead, Task(gps_read, ((10'000'000 * 62) / GPS_BAUD) - 100));
}

void gps_step()
{
	if (gps.newNMEAreceived()) {
		gps.parse(gps.lastNMEA());
	}
}

float gps_get_lat() { return gps.latitudeDegrees; }
float gps_get_lon() { return gps.longitudeDegrees; }
float gps_get_alt() { return gps.altitude; }

void gps_print()
{
	Serial.print('[');
	Serial.print(gps.year);
	Serial.print('-');
	if (gps.month < 10) {
		Serial.print('0');
	}
	Serial.print(gps.month);
	Serial.print('-');
	if (gps.day < 10) {
		Serial.print('0');
	}
	Serial.print(gps.day);
	Serial.print('T');
	if (gps.hour < 10) {
		Serial.print('0');
	}
	Serial.print(gps.hour);
	Serial.print(':');
	if (gps.minute < 10) {
		Serial.print('0');
	}
	Serial.print(gps.minute);
	Serial.print(':');
	if (gps.seconds < 10) {
		Serial.print('0');
	}
	Serial.print(gps.seconds);
	Serial.print('.');
	if (gps.milliseconds < 100) {
		Serial.print('0');
	}
	if (gps.milliseconds < 10) {
		Serial.print('0');
	}
	Serial.print(gps.milliseconds);
	Serial.print(F("] "));

	if (gps.fix) {
		Serial.print(F("Fix("));
		Serial.print((int)gps.fixquality);
		Serial.print(F("): "));
		Serial.print(gps.latitudeDegrees, 4);
		Serial.print(F(", ")); 
		Serial.print(gps.longitudeDegrees, 4);

		Serial.print(F("; Speed: "));
		Serial.print(gps.speed * 0.514444444444446f);  // Convert knots to m/s
		Serial.print(F("m/s; Course: "));
		Serial.print(gps.angle);
		Serial.print(F("deg; Altitude: "));
		Serial.print(gps.altitude);
		Serial.print(F("m; HDOP: "));
		Serial.print(gps.HDOP);
		Serial.print(F("; Satellites: "));
		Serial.println((int)gps.satellites);
	} else {
		Serial.println(F("No fix."));
	}
}
