#include "baro.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <cmath>

/*
 * Baro driver for BMP390
 * Datasheet for the BMP390 can be found at:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf
 */

// BMP390 set register values
#define BMP390_CHIP_ID					(0x60)
#define ENABLE_PRESSURE					(0x01)
#define ENABLE_TEMP						(0x02)
#define ENABLE_SENSOR					(0x30)
#define OSR_x2							(0b001)
#define OSR_x32							(0b101)
// 200Hz / 2^(odr_sel)
#define ODR_12p5_HZ						(0x04)
#define SOFT_RESET						(0xB6)

// BMP390 register addresses
#define BMP390_REG_CHIP_ID				(0x00)
#define BMP390_REG_DATA					(0x04)
#define BMP390_REG_EVENT				(0x10)
#define BMP390_REG_PWR_CTRL				(0x1B)
#define BMP390_REG_OSR					(0x1C)
#define BMP390_REG_ODR					(0x1D)
#define BMP390_REG_IIR					(0x1F)
#define BMP390_REG_CAL					(0x31)
#define BMP390_REG_CMD					(0x7E)

#define BMP_Concat2Bytes(msb, lsb) 		((uint16_t)msb << 8 | (uint16_t)lsb)
#define BMP_Concat3Bytes(msb, lsb, xlsb) ((uint32_t)msb << 16 | (uint32_t)lsb << 8 | (uint32_t)xlsb)

void baro_step();
static float comp_temp(uint32_t uncomp_temp);
static float comp_press(uint32_t uncomp_press);
static void get_calib_data_from_raw(struct BMP390_raw_calib_data *raw_calib_data);

static void spi_start();
static void spi_end();
static uint8_t read_reg(uint8_t reg);
static void write_reg(uint8_t reg, uint8_t data);
static void read_buf(uint8_t reg, uint8_t *data, uint8_t length);

// SPI Settings for BMP390  pg: 3
static const SPISettings spi_settings(10'000'000, MSBFIRST, SPI_MODE0);
static const uint8_t power_conf = (ENABLE_PRESSURE | ENABLE_TEMP | ENABLE_SENSOR);
// Temp [5..3] bytes, Pressure [2..0] bytes
static const uint8_t osr_conf = (OSR_x2 << 3 | OSR_x32);
static const uint8_t odr_conf = ODR_12p5_HZ;

// Variables to store the last used values
static float last_press = NAN;
static float last_alt = NAN;
static int16_t last_temp = INT16_MIN;

static struct BMP390_raw_calib_data {
	uint16_t nvm_par_t1;
	uint16_t nvm_par_t2;
	int8_t 	 nvm_par_t3;
	int16_t  nvm_par_p1;
	int16_t  nvm_par_p2;
	int8_t   nvm_par_p3;
	int8_t   nvm_par_p4;
	uint16_t nvm_par_p5;
	uint16_t nvm_par_p6;
	int8_t   nvm_par_p7;
	int8_t   nvm_par_p8;
	int16_t  nvm_par_p9;
	int8_t   nvm_par_p10;
	int8_t   nvm_par_p11;
} raw_calib_data;

static struct BMP390_calib_data {
	float par_t1;
	float par_t2;
	float par_t3;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float t_lin;
} calib_data;

/*
 * Section 8.4
 * Turns raw calib data into the floating point
 */
static void get_calib_data_from_raw(uint8_t *raw_data)
{
	raw_calib_data.nvm_par_t1 = BMP_Concat2Bytes(raw_data[1], raw_data[0]);
	calib_data.par_t1 = (float)raw_calib_data.nvm_par_t1 / powf(2, -8);
	raw_calib_data.nvm_par_t2 = BMP_Concat2Bytes(raw_data[3], raw_data[2]);
	calib_data.par_t2 = (float)raw_calib_data.nvm_par_t2 / powf(2, 30);
	raw_calib_data.nvm_par_t3 = raw_data[4];
	calib_data.par_t3 = (float)raw_calib_data.nvm_par_t3 / powf(2, 48);
	raw_calib_data.nvm_par_p1 = BMP_Concat2Bytes(raw_data[6], raw_data[5]);
	calib_data.par_p1 = ((float)raw_calib_data.nvm_par_p1 - powf(2, 14)) / powf(2, 20);
	raw_calib_data.nvm_par_p2 = BMP_Concat2Bytes(raw_data[8], raw_data[7]);
	calib_data.par_p2 = ((float)raw_calib_data.nvm_par_p2 - powf(2, 14)) / powf(2, 29);
	raw_calib_data.nvm_par_p3 = raw_data[9];
	calib_data.par_p3 = (float)raw_calib_data.nvm_par_p3 / powf(2, 32);
	raw_calib_data.nvm_par_p4 = raw_data[10];
	calib_data.par_p4 = (float)raw_calib_data.nvm_par_p4 / powf(2, 37);
	raw_calib_data.nvm_par_p5 = BMP_Concat2Bytes(raw_data[12], raw_data[11]);
	calib_data.par_p5 = (float)raw_calib_data.nvm_par_p5 / powf(2, -3);
	raw_calib_data.nvm_par_p6 = BMP_Concat2Bytes(raw_data[14], raw_data[13]);
	calib_data.par_p6 = (float)raw_calib_data.nvm_par_p6 / powf(2, 6);
	raw_calib_data.nvm_par_p7 = raw_data[15];
	calib_data.par_p7 = (float)raw_calib_data.nvm_par_p7 / powf(2, 8);
	raw_calib_data.nvm_par_p8 = raw_data[16];
	calib_data.par_p8 = (float)raw_calib_data.nvm_par_p8 / powf(2, 15);
	raw_calib_data.nvm_par_p9 = BMP_Concat2Bytes(raw_data[18], raw_data[17]);
	calib_data.par_p9 = (float)raw_calib_data.nvm_par_p9 / powf(2, 48);
	raw_calib_data.nvm_par_p10 = raw_data[19];
	calib_data.par_p10 = (float)raw_calib_data.nvm_par_p10 / powf(2, 48);
	raw_calib_data.nvm_par_p11 = raw_data[20];
	calib_data.par_p11 = (float)raw_calib_data.nvm_par_p11 / powf(2, 65);
}

/*
 * Section 8.5 of datasheet
 * Compensates temperature mearsurement to correct for device variance 
 */
static float comp_temp(uint32_t uncomp_temp)
{
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp - calib_data.par_t1);
	partial_data2 = (float)(partial_data1 * calib_data.par_t2);
	/* Update the compensated temperature in calib structure since this is
	 * needed for pressure calculation */
	calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;
	/* Returns compensated temperature */
	return calib_data.t_lin;
}

/*
 * Section 8.6 of datasheet
 * Compensates pressure mearsurement to correct for device variance 
 */
static float comp_press(uint32_t uncomp_press)
{
	/* Variable to store the compensated pressure */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1, partial_data2, partial_data3, partial_data4, partial_out1, partial_out2;
	/* Calibration data */
	partial_data1 = calib_data.par_p6 * calib_data.t_lin;
	partial_data2 = calib_data.par_p7 * (calib_data.t_lin * calib_data.t_lin);
	partial_data3 = calib_data.par_p8 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
	partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib_data.par_p2 * calib_data.t_lin;
	partial_data2 = calib_data.par_p3 * (calib_data.t_lin * calib_data.t_lin);
	partial_data3 = calib_data.par_p4 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
	partial_out2 = (float)uncomp_press * (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data.par_p11;
	comp_press = partial_out1 + partial_out2 + partial_data4;
	return comp_press;
}

/* 
 * Setup the barometer configuration
 * - 32x oversampling for pressure sensor
 * - 2x oversampling for temperature sensor
 * - IIR filter turned off
 * - ODR rate greater than 25 Hz
 */
void baro_setup()
{
	// Read chip id
    if(read_reg(BMP390_REG_CHIP_ID) != BMP390_CHIP_ID) {
        Serial.println(F("BMP390 not found!"));
        abort();
    } else {
        Serial.println(F("BMP390 detected"));
    }

	// Soft reset
	write_reg(BMP390_REG_CMD, SOFT_RESET);
	delay(50);

	// Set settings
	write_reg(BMP390_REG_PWR_CTRL, power_conf);
	write_reg(BMP390_REG_OSR, osr_conf);
	write_reg(BMP390_REG_ODR, odr_conf);
	delay(5);

	// Read settings and calib
	uint8_t raw_calib[21] = { 0 };
	read_buf(BMP390_REG_CAL, raw_calib, 21);
	get_calib_data_from_raw(raw_calib);

	// Test power config
    if (read_reg(BMP390_REG_PWR_CTRL) != power_conf) {
        Serial.println(F("BMP390 power conf is not set!"));
        abort();
    }

    // Test sensor oversampling config
    if (read_reg(BMP390_REG_OSR) != osr_conf) {
        Serial.println(F("BMP390 OSR config not set!"));
        abort();
    }

    // Test sensor output data rate config
    if (read_reg(BMP390_REG_ODR) != odr_conf) {
        Serial.println(F("BMP390 ODR config not set"));
        abort();
    }

	uint8_t is_baro_ready = read_reg(0x03) & 0x70;
	while (is_baro_ready != 0x70) {
		is_baro_ready = read_reg(0x03) & 0x70;
	}

	/* Add the barometer step function to the scheduler
	 */
	scheduler_add(TaskId::Baro, Task(baro_step, KALMAN_PERIOD * 1000L, 250));

	baro_step();
}

/* 
 * Called by the scheduler to get the next barometer values
 */
void baro_step()
{
	// Get the values of uncomp press and temp
	uint8_t raw_data[6];
	read_buf(BMP390_REG_DATA, raw_data, sizeof(raw_data));

	// Get the values into single data types
	uint32_t uncomp_press = BMP_Concat3Bytes(raw_data[2], raw_data[1], raw_data[0]);
	uint32_t uncomp_temp = BMP_Concat3Bytes(raw_data[5], raw_data[4], raw_data[3]);

	// Use the calibration data to get comp pressure
	last_temp = comp_temp(uncomp_temp) * 100;
	last_press = comp_press(uncomp_press);

	// Compute altitude from comp pressure.
	// https://en.wikipedia.org/wiki/Pressure_altitude
	last_alt = 44330 * (1.0f - powf(last_press / 101325, 0.1903f));
}

/* 
 * Returns the last barometer's altitude value in meters
 * Can return NaN if data hasn't been received from the barometer yet.
 */
float baro_get_altitude()
{
	return last_alt;
}

/* 
 * Returns the last barometer's temperature value in degrees centi-Celsius
 */
int16_t baro_get_temp()
{
	return last_temp;
}

/* 
 * Returns the last barometer's pressure value in Pascals
 */
float baro_get_pressure()
{
	return last_press;
}

/* 
 * Prints out the barometer's data for debugging.
 */
void baro_print()
{
	Serial.print(F("Altitude: "));
	Serial.print(last_alt);
	Serial.println('m');

	Serial.print(F("Pressure: "));
	Serial.print(last_press);
	Serial.println("Pa");

	Serial.print(F("Temp: "));
	Serial.print(last_temp / 100);
	Serial.println("C");
}

/* 
 * Start SPI transaction with setting CS pin Low
 */
static void spi_start()
{
    SPI.beginTransaction(spi_settings);
    digitalWrite(PIN_BARO_CS, LOW);
}

/* 
 * End SPI transaction with setting CS pin High
 */
static void spi_end()
{
    digitalWrite(PIN_BARO_CS, HIGH);
    SPI.endTransaction();
}

/*
 * Reads from a single register
 */
static uint8_t read_reg(uint8_t reg)
{
    spi_start();

    SPI.transfer(reg | 0x80);
    SPI.transfer(0);
    uint8_t res = SPI.transfer(0);

    spi_end();

    return res;
}

/*
 * Writes a single register
 */
static void write_reg(uint8_t reg, uint8_t data)
{
    spi_start();
    
    SPI.transfer(reg & ~0x80);
    SPI.transfer(data);

    spi_end();
}

/*
 * Reads from a multiple registers on the barometer
 * The read starts from the given register and the length bytes afterwards.
 */
static void read_buf(uint8_t reg, uint8_t *data, uint8_t length)
{
    spi_start();

    SPI.transfer(reg | 0x80);
    SPI.transfer(0);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0);
    }
    
    spi_end();
}
