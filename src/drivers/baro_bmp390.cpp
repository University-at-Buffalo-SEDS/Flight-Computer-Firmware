#include "baro.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP3XX.h>

#include <cmath>

// Constants
#define SEALEVELPRESSURE_HPA (1013.25)

// Function definitions
void baro_setup();
void baro_step();
float baro_get_altitude();
int16_t baro_get_temp();
float baro_get_pressure();
void baro_print();

// File-Global variables
static float last_press = NAN;
static float last_alt = NAN;
static int16_t last_temp = INT16_MIN;

Adafruit_BMP3XX bmp;

// Global functions
void baro_setup() {

	if (!bmp.begin_SPI(PIN_BARO_CS)) {
		Serial.println("Could not find a vaild BMP3 sensor, check wiring!");
		abort();
	}

	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
	bmp.setOutputDataRate(BMP3_ODR_25_HZ);

	scheduler_add(TaskId::Baro, Task(baro_step, KALMAN_PERIOD * 1000L, 160));
}

void baro_step() {
	last_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	last_press = bmp.pressure;
	last_temp = bmp.temperature;
}

float baro_get_altitude() {
	return last_alt;
}

int16_t baro_get_temp() {
	return last_press;
}

float baro_get_pressure() {
	return last_temp;
}

void baro_print()
{
	Serial.print(F("Altitude: "));
	Serial.print(last_alt);
	Serial.println('m');

	Serial.print(F("Pressure: "));
	Serial.print(last_press);
	Serial.println("Pa");
}