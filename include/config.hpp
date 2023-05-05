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

// Time that passes between calls to kalman_step (in ms).
// Note: may have to reconfigure sensors to supply data faster if this is changed.
#define KALMAN_PERIOD 100

// Altitude and velocity and acceleration below which landing will
// be detected.  Acceleration is comphensated for gravity.
// In order to avoid issues with spurious readings, these values must
// be maintained for at least LANDED_TIME_MS before landing is detected.
// Descent velocity with main should be about 6 m/s
#define LANDED_ALT -10
#define LANDED_VELOCITY 2
#define LANDED_ACCEL 1
#define LANDED_TIME_MS 5000

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

// We need to have enough samples that a launch event will be detected
// before this overflows.  The default value coresponds to one second
// worth of samples.  With the double delay this means that the estimate
// will have a 1-2 second delay.
#define EST_HISTORY_SAMPLES (1000/KALMAN_PERIOD)

// Number of milliseconds of log data write out from before launch is detected.
#define PRELOG_MS 2000

// Pin definitions 
#define PIN_BMI088_Accel_CS 	PB11
#define PIN_BMI088_Gyro_CS		PB12
#define PIN_BMP390_CS			PB13
#define PIN_FLASH_CS   			PB8
  
// How long to leave the fire channels active, in milliseconds
#define CHANNEL_FIRE_TIME 1000

#define EEPROM_FLIGHT 0

// Standard gravity in m/s^2
#define STANDARD_GRAVITY 9.80665f

#define DEBUG 0