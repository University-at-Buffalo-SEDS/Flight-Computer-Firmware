#include "accel.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <SPI.h>

/*
 * Accel driver for BMI088
 * Datasheet for the BMI088 can be found at:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
 */

// Page 25
// BMI088 Accel values
#define BMI088_ACC_CHIP_ID              (0x1E)
#define BMI088_ACC_24G_RANGE            (0x03)
#define BMI088_ACC_BWP_OSR4             (0x80)
#define BMI088_ACC_ODR_200Hz            (0x09)

// BMI088 register addresses
#define BMI088_ACC_REG_CHIP_ID          (0x00)
#define BMI088_ACC_REG_DATA             (0x12)
#define BMI088_ACC_REG_CONF             (0x40)
#define BMI088_ACC_REG_RANGE            (0x41)
#define BMI088_ACC_REG_PWR_CONF         (0x7C)
#define BMI088_ACC_REG_PWR_CTRL         (0x7D)
#define BMI088_ACC_REG_SOFTRESET        (0x7E)

void accel_step();

static void spi_start();
static void spi_end();
static uint8_t read_reg(uint8_t reg);
static void write_reg(uint8_t reg, uint8_t data);
static void read_buf(uint8_t reg, uint8_t *data, uint8_t length);

// SPI Settings for BMI088   pg: 46
static const SPISettings spi_settings(10'000'000, MSBFIRST, SPI_MODE0);
// XYZ accelation values
static float last_accel[3] = {NAN, NAN, NAN};

// Device config settings
static const uint8_t range_conf = BMI088_ACC_24G_RANGE;
static const uint8_t sampling_conf = (BMI088_ACC_BWP_OSR4 | BMI088_ACC_ODR_200Hz);

/*
 * Verifies the accel device is connected and sets accel configuration.
 * Also adds the accel_step to the scheduler.
 */
void accel_setup()
{
    // All delays are based on Section 4.6.1 of the datasheet
    // Requires a rising edge on CS to startup SPI on accel
    digitalWrite(PIN_ACCEL_CS, LOW);
    delay(1);
    digitalWrite(PIN_ACCEL_CS, HIGH);
    // Wait 50 milliseconds for for CS high
    delay(50);

    // Do a soft reset on device.
    write_reg(BMI088_ACC_REG_SOFTRESET, 0xB6);
    // Wait 50 milliseconds for softreset
    delay(50);

    // Dummy read
    read_reg(BMI088_ACC_REG_CHIP_ID);
    // Check if we can get the chip id
    if(read_reg(BMI088_ACC_REG_CHIP_ID) != BMI088_ACC_CHIP_ID) {
        Serial.println(F("BMI088 Accel not found!"));
        abort();
    } else {
        Serial.println(F("BMI088 Accel detected"));
    }

    // Put the Accel into Active mode
    write_reg(BMI088_ACC_REG_PWR_CONF, 0x00);
    // Section 5.3.10
    // Set for max oversampling with total sampling rate > kalman frequency
    write_reg(BMI088_ACC_REG_CONF, sampling_conf);
    // Section 5.3.11
    // Set for 24Gs
    write_reg(BMI088_ACC_REG_RANGE, range_conf);
    // Turns on the accelerometer's sensor module
    write_reg(BMI088_ACC_REG_PWR_CTRL, 0x04);
    delay(50);

    // Test sampling config
    if (read_reg(BMI088_ACC_REG_CONF) != sampling_conf) {
        Serial.println(F("BMI088 Accel incorrect sampling rate set!"));
        abort();
    }

    // Test range config
    if (read_reg(BMI088_ACC_REG_RANGE) != range_conf) {
        Serial.println(F("BMI088 Accel incorrect range set!"));
        abort();
    }

    // Test if accel is powered on
    if (read_reg(BMI088_ACC_REG_PWR_CTRL) != 0x04) {
        Serial.println(F("BMI088 Accel did not turn on!"));
        abort();
    }

    scheduler_add(TaskId::Accel, Task(accel_step, KALMAN_PERIOD * 1000L, 120));

    accel_step();
}

/* 
 * Called by the scheduler to get the next accelerometer values
 */
void accel_step()
{
    uint8_t raw_data[6];
    read_buf(BMI088_ACC_REG_DATA, raw_data, sizeof(raw_data));

    // Computations based on page 27
    int16_t raw_x = (raw_data[1] << 8) + raw_data[0];
    int16_t raw_y = (raw_data[3] << 8) + raw_data[2];
    int16_t raw_z = (raw_data[5] << 8) + raw_data[4];

    // Use to go from LSB to Gees
    static const float BMI088_MULTIPLER = 1.0f/(1<<15) * (1<<(range_conf + 1)) * 1.5f;

    // LSB * (Gees/LSB) * (m/s^2 / Gees) 
    last_accel[0] = (float)raw_x * BMI088_MULTIPLER * STANDARD_GRAVITY;
    last_accel[1] = (float)raw_y * BMI088_MULTIPLER * STANDARD_GRAVITY;
    last_accel[2] = (float)raw_z * BMI088_MULTIPLER * STANDARD_GRAVITY;
}

/* 
 * Returns the last read accelerometer's XYZ acceleration value in m/s^2
 */
float *accel_get()
{
    return last_accel;
}

/* 
 * Prints out the accelerometer's data for debugging.
 */
void accel_print()
{
	float *a = accel_get();
	Serial.print(F("Accel: "));
	Serial.print(a[0]);
	Serial.print(F(", "));
	Serial.print(a[1]);
	Serial.print(F(", "));
	Serial.print(a[2]);
	Serial.print(F(" ("));
	Serial.print(sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
	Serial.println(F(") m/s^2"));
}

/* 
 * Start SPI transaction with setting CS pin Low
 */
static void spi_start()
{
    SPI.beginTransaction(spi_settings);
    digitalWrite(PIN_ACCEL_CS, LOW);
}

/* 
 * End SPI transaction with setting CS pin High
 */
static void spi_end()
{
    digitalWrite(PIN_ACCEL_CS, HIGH);
    SPI.endTransaction();
}

/*
 * Reads from a single register on the accel
 * Based on page 48
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
 * Writes a single register on the accel
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
 * Reads from a multiple registers on the accel
 * The read starts from the given register and goes for the value of length bytes afterwards.
 * Based on page 48 of datasheet
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