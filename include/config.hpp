#pragma once
#ifdef NATIVE_TEST
#define PC0 0
#define PC1 0
#else
#include <Arduino.h>
#endif

#include <array>
#include <cstddef>

#define LOG_ENABLE 1

// Rates which trigger a launch event (deployment will only occur
// when apogee is detected after a launch event).  Velocity and
// acceleration must be greater than both of these for a launch
// event to be detected.
#define LAUNCH_VELOCITY 8
#define LAUNCH_ACCEL 20

// Altitude and velocity and acceleration below which landing will
// be detected.  Acceleration is comphensated for gravity.
// In order to avoid issues with spurious readings, these values must
// be maintained for at least LANDED_TIME_MS before landing is detected.
// Descent velocity with main should be about 6 m/s
#define LANDED_ALT 30
#define LANDED_VELOCITY 2
#define LANDED_ACCEL 1
#define LANDED_TIME_MS 5000

// If the downward velocity goes above this value after drogue
// deployment then the main will deploy.  This is a failsafe
// in case the drogue fails to deploy.
// Initial estimates are that drogue descent will be at about 38 m/s.
// XXX: This value should be checked before use.
//#define FAILSAFE_VELOCITY 60

// Time that passes between calls to kalman_step (in ms).
// Note: may have to reconfigure sensors to supply data faster if this is changed.
#define KALMAN_PERIOD 100

// Noise values for Kalman filter.  These should be rounded up a bit.
// Standard deviation of altimeter noise in m
//#define ALTITUDE_SIGMA 1.5f  // For MPL3115A2
//#define ALTITUDE_SIGMA 1.0f  // For BMP280.  Note: actual value is about 0.16
#define ALTITUDE_SIGMA 1.0f  // For BMP390   actual value is about 0.9 without IIR filtering
// Standard deviation of accelerometer noise in m/s^2
#define ACCELERATION_SIGMA 1.0f  // For LSM9DS0 and ADXL345  Note: actual value is about 0.05.
// Standard deviation of model noise in m/s^2
#define MODEL_SIGMA 0.2f

// Precomputed Kalman gains.  Reduces startup time, but must be recomputed for every change of the parameters.
//#define KALMAN_GAINS 0, 0, 0, 0, 0, 0

#define LED_BUILTIN				PB5
#define PIN_BMI088_Accel_CS 	PB12
#define PIN_BMP390_CS			PB13
#define PIN_FLASH_CS   			PB14
  
// How long to leave the fire channels active, in milliseconds
#define CHANNEL_FIRE_TIME 1000

struct ChannelConfig {
	int fire_pin;
};

enum class Channel {
	Drogue,
	Main,
	Count
};

constexpr std::array<ChannelConfig, (size_t)Channel::Count> channel_config = {
	ChannelConfig {0},
	ChannelConfig {0}
};

// #define FLIGHT_FLASH_MOSI_PIN PB15
// #define FLIGHT_FLASH_MISO_PIN PB14
// #define FLIGHT_FLASH_SCK_PIN PB13

#define GPS_SERIAL Serial1
#define XBEE_SERIAL Serial3

#define EEPROM_FLIGHT 0

#define GPS_UPDATE_RATE 5

// Standard gravity in m/s^2
#define STANDARD_GRAVITY 9.80665f

#define DEBUG 0

// Time to wait before testing pyrochannels
#define TEST_FIRE_DELAY 2000
#define TEST_FIRE_UPTIME 1000