#include "baro.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <SPI.h>

#include <cmath>

struct BMPCal {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	int32_t t_fine;
} calib;

#define BMP280_CHIPID 0x58
enum BmpReg : uint8_t {
	REG_CHIPID = 0xD0,
	REG_CONTROL = 0xF4,
	REG_CONFIG = 0xF5,
	REG_CALIB_START = 0x88,
	REG_READOUT_START = 0xF7,
};

static const SPISettings spi_settings(10'000'000, MSBFIRST, SPI_MODE0);
static float last_press = NAN;
static float last_alt = NAN;
static int16_t last_temp = INT16_MIN;


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t compensate_temp(int32_t adc_T)
{
	int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
	int32_t var2  = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
	calib.t_fine = var1 + var2;
	return (calib.t_fine * 5 + 128) >> 8;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t compensate_press(int32_t adc_P)
{
	int64_t var1 = ((int64_t)calib.t_fine) - 128000;
	int64_t var2 = var1 * var1 * (int64_t)calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
	if (var1 == 0) {
		return 0;
	}
	int64_t p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
	return (uint32_t)p;
}

static void spi_begin()
{
	SPI.beginTransaction(spi_settings);
	digitalWrite(PIN_BMP280_CS, LOW);
}

static void spi_end()
{
	digitalWrite(PIN_BMP280_CS, HIGH);
	SPI.endTransaction();
}

static uint8_t read8(uint8_t reg)
{
	spi_begin();
	SPI.transfer(reg | 0x80);
	uint8_t v = SPI.transfer(0);
	spi_end();
	return v;
}

static void write8(uint8_t reg, uint8_t value)
{
	spi_begin();
	SPI.transfer(reg & ~0x80);
	SPI.transfer(value);
	spi_end();
}

static void read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
	spi_begin();
	SPI.transfer(reg | 0x80);
	for (uint8_t i = 0; i < len; ++i) {
		buf[i] = SPI.transfer(0);
	}
	spi_end();
}

static void baro_step()
{
	uint8_t data[6];
	read_buf(REG_READOUT_START, data, 6);
	uint32_t uncomp_p = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
	uint32_t uncomp_t = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));

	// Ignore bogus data which we occasionally get on startup
	if (uncomp_p == (1<<19) || uncomp_t == (1<<19)) {
		return;
	}

	// Temp comes first because it updates t_fine.
	int32_t temp = compensate_temp(uncomp_t);
	uint32_t press = compensate_press(uncomp_p);

	if (press != 0) {
		last_press = (int32_t)(press / 256);
		last_alt = 44330 * (1.0f - powf(last_press / 101325, 0.1903f));
	}
	last_temp = temp;
}

void baro_setup()
{
	pinMode(PIN_BMP280_CS, OUTPUT);
	digitalWrite(PIN_BMP280_CS, HIGH);

	if (read8(REG_CHIPID) != BMP280_CHIPID) {
		Serial.println(F("Failed to set up BMP280!"));
		while (true) { delay(1); }
	} else {
		Serial.println(F("BMP280 detected."));
	}

	read_buf(REG_CALIB_START, (uint8_t*)&calib, 24);

	write8(REG_CONTROL, 0b010'101'11); // osrs_t=2(2x), osrs_p=5(16x), mode=3(normal)
	write8(REG_CONFIG, 0b000'000'0'0);  // t_sb=0(0.5ms), filter=0, resv=0, spi3w_en=0

	scheduler_add(TaskId::Baro, Task(baro_step, KALMAN_PERIOD * 1000L, 160));
}

float baro_get_altitude()
{
	return last_alt;
}

int16_t baro_get_temp()
{
	return last_temp;
}

void baro_print()
{
	Serial.print(F("Altitude: "));
	Serial.print(last_alt);
	Serial.println('m');
}
