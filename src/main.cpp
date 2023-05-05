#include "accel.hpp"
#include "baro.hpp"
#include "config.hpp"
#include "kalman.hpp"
#include "log.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <SPI.h>
#include <Wire.h>

#include <cmath>

#if defined (USBCON) && defined(USBD_USE_CDC)
#include "USBSerial.h"
USBSerial usb_serial;
#endif

struct ChannelStatus {
	uint32_t fire_time;
	bool firing;
};

// Prototypes
void command_step();
void blink_step();
void print_step();
void deployment_step();

#ifdef KALMAN_GAINS
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f, {KALMAN_GAINS});
#else
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f,
		ALTITUDE_SIGMA, ACCELERATION_SIGMA, MODEL_SIGMA);
#endif

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	// pinMode(PIN_BATT_V, INPUT_ANALOG);
	// pinMode(PIN_SYS_V, INPUT_ANALOG);
	analogReadResolution(12);  // Enable full resolution

#if defined (USBCON) && defined(USBD_USE_CDC)
	usb_serial.begin();
#else
	Serial.begin(9'600);
#endif

	Serial.println(F("Flight Computer " __DATE__ " " __TIME__));

	Wire.begin();

	SPI.begin();

	baro_setup();
	accel_setup();
#if LOG_ENABLE
	log_setup();
#endif

	scheduler_add(TaskId::Deployment, Task(deployment_step, KALMAN_PERIOD * 1000L, 2500));
	scheduler_add(TaskId::Command, Task(command_step, 100'000L, 10));
	scheduler_add(TaskId::Print, Task(print_step, 3'000'000L, 3000));
	scheduler_add(TaskId::Blink, Task(blink_step, (KALMAN_PERIOD / 2) * 1000L, 20));
	
}

void loop()
{
	uint32_t wait_time = schedule();
	if (wait_time > 4) {
		delayMicroseconds(wait_time - 4);
	}
}

void command_step()
{
	switch (Serial.read()) {
	case 'r':
		log_print_all();
		break;
	default:
		// Serial.println("Unrecognized command.");
		break;
	}
}

void blink_step()
{
	static bool on = false;
	digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
	on = !on;
}

void print_step()
{
	accel_print();
	baro_print();
}

void deployment_step() {
	static uint32_t land_time = 0;
	static uint32_t buffer_time_ms = 60 * 1000; // Give a time to prevent preemptive landed readings
	static FlightPhase phase = FlightPhase::Startup;
	static AvgHistory<float, EST_HISTORY_SAMPLES, 3> gravity_est_state;
	static AvgHistory<float, EST_HISTORY_SAMPLES, 3> ground_level_est_state;
	static kfloat_t apogee = 0;
	float *accel = accel_get();
	float raw_alt = baro_get_altitude();

	// Current system time since boot
	uint32_t step_time = millis();

	if (std::isnan(raw_alt) || std::isnan(accel[0])) {
		// Wait until the next run, by which time we may have new data.
		return;
	}

	// Kalman filter only needs the magitude of accel
	float accel_mag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

	if (phase < FlightPhase::Dropped) {
		gravity_est_state.add(accel_mag);
		ground_level_est_state.add(raw_alt);
	}

	if (phase == FlightPhase::Startup) {
		if (!ground_level_est_state.full() ||
				!gravity_est_state.full()) {
			return;
		}
		
		phase = FlightPhase::Dropped;

		buffer_time_ms += step_time;

#if LOG_ENABLE
		log_start();
#endif
	}

	accel_mag -= gravity_est_state.old_avg();
	float alt = raw_alt - ground_level_est_state.old_avg();

	// Run the kalman filter with accel and alt
	kf.step(accel_mag, alt);

	if (phase == FlightPhase::Dropped) {

		if (kf.pos() < LANDED_ALT &&
				abs(kf.rate()) < LANDED_VELOCITY &&
				abs(kf.accel()) < LANDED_ACCEL &&
				(step_time > buffer_time_ms)) {
			
			
			if (land_time == 0) {
				land_time = step_time;
				buffer_time_ms = 0;
				if (land_time == 0) {
					land_time = 1;
				}
			} else if (delta(land_time, step_time) > LANDED_TIME_MS) { // Must stay landed long enough
				
				phase = FlightPhase::Landed;
#if LOG_ENABLE
				log_stop();
#endif
			}
		} else {
			land_time = 0;
		}

	}

	// Log the current data if enabled
#if LOG_ENABLE
	log_add(LogMessage(
		step_time,
		kf.state(),
		raw_alt,
		accel[0],
		accel[1],
		accel[2],
		baro_get_temp(),
		baro_get_pressure()
	));
#endif
}
