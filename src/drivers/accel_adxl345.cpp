#include "accel.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <SPI.h>

// Adafruit library does not support hardware SPI and reads the
// axes one at a time, contrary to what the datasheet recommends.

// Might be unsafe if anything else is using the same SPI bus.
// May also cause unexpected delays when the interrupt triggers.
#define ADXL345_USE_INT 0

static const SPISettings spi_settings(5'000'000, MSBFIRST, SPI_MODE3);
static float last_accel[3] = {NAN, NAN, NAN};

void accel_step();

static void write_reg(uint8_t reg, uint8_t data)
{
	digitalWrite(PIN_ADXL345_CS, LOW);
	SPI.transfer(reg);
	SPI.transfer(data);
	digitalWrite(PIN_ADXL345_CS, HIGH);
}

static uint8_t read_reg(uint8_t reg)
{
	digitalWrite(PIN_ADXL345_CS, LOW);
	SPI.transfer(reg | 0x80);
	uint8_t res = SPI.transfer(0);
	digitalWrite(PIN_ADXL345_CS, HIGH);
	return res;
}

static void read_buf(uint8_t reg, uint8_t *data, uint8_t length)
{
	digitalWrite(PIN_ADXL345_CS, LOW);
	SPI.transfer(reg | 0x80 | 0x40);
	for (uint8_t i = 0; i < length; ++i) {
		data[i] = SPI.transfer(0);
	}
	digitalWrite(PIN_ADXL345_CS, HIGH);
}

#define ADXL345_REG_DEVID               (0x00)    // Device ID
#define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
#define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
#define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
#define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
#define ADXL345_REG_DUR                 (0x21)    // Tap duration
#define ADXL345_REG_LATENT              (0x22)    // Tap latency
#define ADXL345_REG_WINDOW              (0x23)    // Tap window
#define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
#define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
#define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
#define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
#define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
#define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
#define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
#define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
#define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
#define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
#define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
#define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
#define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
#define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
#define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
#define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
#define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
#define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
#define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
#define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
#define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
#define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status

#define ADXL345_FULL_SCALE_MULTIPLIER ((2 * 2) / 1024.0f)

void accel_setup()
{
#if ADXL345_USE_INT
	pinMode(PIN_ADXL345_INT, INPUT_PULLUP);
#endif
	pinMode(PIN_ADXL345_CS, OUTPUT);
	digitalWrite(PIN_ADXL345_CS, HIGH);

	SPI.beginTransaction(spi_settings);

	if (read_reg(ADXL345_REG_DEVID) != 0xE5) {
		Serial.println(read_reg(ADXL345_REG_DEVID));
		Serial.println(F("ADXL345 not found!"));
		abort();
	} else {
		Serial.println(F("ADXL345 detected."));
	}

	// Set measure bit
	write_reg(ADXL345_REG_POWER_CTL, 0b0000'1000);

	write_reg(ADXL345_REG_DATA_FORMAT, 0b0010'1011); // 16G full range
	write_reg(ADXL345_REG_BW_RATE, 0b0000'0111); // 12.5Hz update rate

#if ADXL345_USE_INT
	write_reg(ADXL345_REG_INT_MAP, 0b0000'0000); // All interrupts to INT1
	write_reg(ADXL345_REG_INT_ENABLE, 0b1000'0000); // Enable data ready interrupt
	SPI.endTransaction();
	attachInterrupt(digitalPinToInterrupt(PIN_ADXL345_INT), accel_step, FALLING);
#else
	SPI.endTransaction();
	scheduler_add(TaskId::Accel, Task(accel_step, KALMAN_PERIOD * 1000L, 120));
#endif

	accel_step();
}

// Takes less than 50us.
void accel_step()
{
	uint8_t data[6];
	SPI.beginTransaction(spi_settings);
	read_buf(ADXL345_REG_DATAX0, data, sizeof(data));
	SPI.endTransaction();

	int16_t raw_x = word(data[1], data[0]);
	int16_t raw_y = word(data[3], data[2]);
	int16_t raw_z = word(data[5], data[4]);
	last_accel[0] = (float)raw_x * ADXL345_FULL_SCALE_MULTIPLIER * STANDARD_GRAVITY;
	last_accel[1] = (float)raw_y * ADXL345_FULL_SCALE_MULTIPLIER * STANDARD_GRAVITY;
	last_accel[2] = (float)raw_z * ADXL345_FULL_SCALE_MULTIPLIER * STANDARD_GRAVITY;
}

// Returned values must be in m/s^2
float *accel_get() {
	return last_accel;
}

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
