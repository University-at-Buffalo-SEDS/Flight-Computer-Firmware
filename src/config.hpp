#pragma once

#define LOG_ENABLE 1

// Rates which trigger a launch event (deployment will only occur
// when apogee is detected after a launch event).  Velocity and
// acceleration must be greater than both of these for a launch
// event to be detected.
#define LAUNCH_VELOCITY 8
#define LAUNCH_ACCEL 20


// Altitude and velocity and acceleration below which landing will
// be detected.  Acceleration is comphensated for gravity.
// Descent velocity with main should be about 6 m/s
#define LANDED_ALT 30
#define LANDED_VELOCITY 3
#define LANDED_ACCEL 2

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
// Standard deviation of altimiter noise in m
//#define ALTITUDE_SIGMA 1.5  // For MPL3115A2
#define ALTITUDE_SIGMA 1  // For BMP280.  Note: actual value is about 0.16
// Standard deviation of accelerometer noise in m/s^2
#define ACCELERATION_SIGMA 1  // For LSM9DS0 and ADXL345  Note: actual value is about 0.05.
// Standard deviation of model noise in m/s^2
#define MODEL_SIGMA 0.2

// Precomputed Kalman gains.  Reduces startup time, but must be recomputed for every change of the parameters.
//#define KALMAN_GAINS 0, 0, 0, 0, 0, 0

// We need to have enough samples that a launch event will be detected
// before this overflows.  The default value coresponds to one second'
// worth of samples.  With the double delay this means that the estimate
// will have a 1-2 second delay.
#define DELAYED_EST_SAMPLES (1000/KALMAN_PERIOD)

// Altitude at which the main parachute should be deployed, in meters.
#define MAIN_DEPLOY_ALTITUDE 381  // 1250ft

// Battery voltage level reading configuration.
// TODO: Better numbers for these.
// The value that the analog input reports when the battery is at 0V.
#define BATT_MIN_READING 0
// The value that the analog input reports when the battery is full.
#define BATT_FULL_READING 41350
// The voltage that the battery is actually at when full (in millivolts).
#define BATT_FULL_VOLTAGE 8320

// Number of milliseconds of log data write out from before launch is detected.
#define PRELOG_MS 2000

#define PIN_ADXL345_CS 24
#define PIN_BMP280_CS 25
#define PIN_FLASH_CS 29
#define PIN_LAUNCH 26
#define PIN_DROGUE 27
#define PIN_MAIN 28
#define PIN_BATT_V A1

#define EEPROM_FLIGHT 0

// These are from the perspective of the XBee module. i.e. PIN_XBEE_CTS is the Teensy's RTS pin and is connected to the XBee's CTS pin.
// Any digital pin.
#define PIN_XBEE_CTS 22
// Must be 23 for Serial2 (The Teensy only supports CTS on pin 23 for Serial2).
#define PIN_XBEE_RTS 23

#define GPS_UPDATE_RATE 5

// Standard gravity in m/s^2
#define STANDARD_GRAVITY 9.80665

#define DEBUG 0
