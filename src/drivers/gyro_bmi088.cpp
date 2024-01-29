#include "gyro.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <SPI.h>

/*
 * Gyro driver for BMI088
 * Datasheet for the BMI088 can be found at:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 */

// Page 36
// BMI088 Gyro values
#define BMI088_GYR_CHIP_ID              (0x0F)
#define BMI088_GYR_2000DPS_RANGE        (0x00)
#define BMI088_GYR_2000DPS_RES_LSB_DPS  (16.384f)
#define BMI088_GYR_ODR_100Hz_BW_32Hz    (0x07)

// BMI088 register addresses
#define BMI088_GYR_REG_CHIP_ID          (0x00)
#define BMI088_GYR_REG_DATA             (0x02)
#define BMI088_GYR_REG_RANGE            (0x0F)
#define BMI088_GYR_REG_BANDWIDTH        (0x10)
#define BMI088_GYR_REG_LPM1             (0x11)
#define BMI088_GYR_REG_SOFTRESET        (0x14)

void gyro_step();

static void spi_start();
static void spi_end();
static uint8_t read_reg(uint8_t reg);
static void write_reg(uint8_t reg, uint8_t data);
static void read_buf(uint8_t reg, uint8_t *data, uint8_t length);

// SPI Settings for BMI088   pg: 46
static const SPISettings spi_settings(10'000'000, MSBFIRST, SPI_MODE0);
// XYZ angular velocity values in degrees per second
static float last_gyro[3] = {NAN, NAN, NAN};

/*
 * Verifies the gyro device is connected and sets gyro configuration.
 * Also adds the gyro_step to the scheduler.
 */
void gyro_setup()
{
    // Check if we can get the chip id
    if(read_reg(BMI088_GYR_REG_CHIP_ID) != BMI088_GYR_CHIP_ID) {
        Serial.println(F("BMI088 Gyro not found!"));
        abort();
    } else {
        Serial.println(F("BMI088 Gyro detected"));
    }

    // Do a soft reset on device.
    write_reg(BMI088_GYR_REG_SOFTRESET, 0xB6);
    // Wait 50 milliseconds for softreset
    delay(50);

    write_reg(BMI088_GYR_REG_RANGE, BMI088_GYR_2000DPS_RANGE);
    write_reg(BMI088_GYR_REG_BANDWIDTH, BMI088_GYR_ODR_100Hz_BW_32Hz);
    delay(50);

    // Test sampling config
    if (read_reg(BMI088_GYR_REG_RANGE) != BMI088_GYR_2000DPS_RANGE) {
        Serial.println(F("BMI088 Gyri incorrect range set!"));
        abort();
    }

    // Test range config
    if (read_reg(BMI088_GYR_REG_BANDWIDTH) != BMI088_GYR_ODR_100Hz_BW_32Hz) {
        Serial.println(F("BMI088 Gyro incorrect bandwidth set!"));
        abort();
    }

    scheduler_add(TaskId::Gyro, Task(gyro_step, KALMAN_PERIOD * 1000L, 120));

    gyro_step();
}

/* 
 * Returns the last read gyroscope's XYZ acceleration value in m/s^2
 */
void gyro_step() {
    uint8_t raw_data[6];
    read_buf(BMI088_GYR_REG_DATA, raw_data, sizeof(raw_data));

    // Computations based on page 37
    int16_t raw_x = (raw_data[1] << 8) + raw_data[0];
    int16_t raw_y = (raw_data[3] << 8) + raw_data[2];
    int16_t raw_z = (raw_data[5] << 8) + raw_data[4];

    // Convert from raw LSB to degrees per second
    last_gyro[0] = (float)raw_x / BMI088_GYR_2000DPS_RES_LSB_DPS;
    last_gyro[1] = (float)raw_y / BMI088_GYR_2000DPS_RES_LSB_DPS;
    last_gyro[2] = (float)raw_z / BMI088_GYR_2000DPS_RES_LSB_DPS;
}

float *gyro_get() {
    return last_gyro;
}

/* 
 * Prints out the gyroscope's data for debugging.
 */
void gyro_print()
{
	float *r = gyro_get();
	Serial.print(F("Gyro: "));
	Serial.print(r[0]);
	Serial.print(F(", "));
	Serial.print(r[1]);
	Serial.print(F(", "));
	Serial.print(r[2]);
	Serial.println(F(" degree/s"));
}

/* 
 * Start SPI transaction with setting CS pin Low
 */
static void spi_start()
{
    SPI.beginTransaction(spi_settings);
    digitalWrite(PIN_GYRO_CS, LOW);
}

/* 
 * End SPI transaction with setting CS pin High
 */
static void spi_end()
{
    digitalWrite(PIN_GYRO_CS, HIGH);
    SPI.endTransaction();
}

/*
 * Reads from a single register on the gyro
 * Based on page 47
 */
static uint8_t read_reg(uint8_t reg)
{
    spi_start();

    SPI.transfer(reg | 0x80);
    uint8_t res = SPI.transfer(0);

    spi_end();

    return res;
}

/*
 * Writes a single register on the gyro
 * Based on page 47
 */
static void write_reg(uint8_t reg, uint8_t data)
{
    spi_start();
    
    SPI.transfer(reg);
    SPI.transfer(data);

    spi_end();
}

/*
 * Reads from a multiple registers on the gyro
 * The read starts from the given register and goes for the value of length bytes afterwards.
 * Based on page 47 of datasheet
 */
static void read_buf(uint8_t reg, uint8_t *data, uint8_t length)
{
    spi_start();

    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0);
    }
    
    spi_end();
}