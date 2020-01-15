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
// Standard deviation of altimiter noise in m
//#define ALTITUDE_SIGMA 1.5f  // For MPL3115A2
#define ALTITUDE_SIGMA 1.0f  // For BMP280.  Note: actual value is about 0.16
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
#define DELAYED_EST_SAMPLES (1000/KALMAN_PERIOD)

// Altitude at which the main parachute should be deployed, in meters.
#define MAIN_DEPLOY_ALTITUDE 381.0f  // 1250ft

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

#define PIN_ADXL345_CS PA8
#define PIN_BMP280_CS PA9
#define PIN_FLASH_CS PB11
//#define PIN_LAUNCH
#define PIN_DROGUE PC0
#define PIN_MAIN PC1
#define PIN_BATT_V A0

#define FLIGHT_FLASH_MOSI_PIN PB15
#define FLIGHT_FLASH_MISO_PIN PB14
#define FLIGHT_FLASH_SCK_PIN PB13

#define GPS_SERIAL Serial1
#define XBEE_SERIAL Serial2

#define EEPROM_FLIGHT 0

#define GPS_UPDATE_RATE 5

// Standard gravity in m/s^2
#define STANDARD_GRAVITY 9.80665f

#define DEBUG 0
