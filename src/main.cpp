#include "baro.hpp"
#include "accel.hpp"
#include "gps.hpp"
#include "kalman.hpp"
#include "sd.hpp"
#include "config.hpp"
#include "util.hpp"
#include "scheduler.hpp"
#include "radio.hpp"

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <ADC.h>

// Prototypes
void blink_step();
void print_step();
void deployment_step();

ADC adc;

void setup()
{

	pinMode(PIN_DROGUE, OUTPUT);
	digitalWrite(PIN_DROGUE, LOW);

	pinMode(PIN_MAIN, OUTPUT);
	digitalWrite(PIN_MAIN, LOW);

	pinMode(PIN_LAUNCH, OUTPUT);
	digitalWrite(PIN_LAUNCH, LOW);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	pinMode(PIN_BATT_V, INPUT);

	Serial.begin(1'000'000);
	//while (!Serial) delay(1);

	Serial.println(F("Avionics starting!"));

	Wire.begin();

	SPI.setSCK(14);
	SPI.begin();

	adc.setAveraging(16);
	adc.setResolution(16);
	adc.setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
	adc.setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
	adc.startContinuous(PIN_BATT_V, ADC_0);

	gps_setup();
	baro_setup();
	accel_setup();
#if SD_LOG
	sd_setup();
#endif
	radio_setup();
	kalman_setup();

	scheduler_add(TaskId::Deployment, Task(deployment_step, KALMAN_PERIOD * 1000L));
	scheduler_add(TaskId::Print, Task(print_step, 3'000'000L));
	scheduler_add(TaskId::Blink, Task(blink_step, (KALMAN_PERIOD / 2) * 1000L));
}

void loop()
{
	uint32_t next_run = scheduler();
	int32_t wait_time = sdelta(micros(), next_run);
	if (wait_time > 1) {
		delayMicroseconds(wait_time - 1);
	}
}


void blink_step()
{
	static bool on = 0;
	digitalWrite(LED_BUILTIN, on);
	on = !on;
}

void print_step()
{
	gps_print();
	accel_print();
	baro_print();
}


void deployment_step()
{
	static uint32_t drogue_trigger_time = 0;
	static uint32_t main_trigger_time = 0;
	static uint32_t land_time = 0;
	static FlightPhase phase = FlightPhase::Startup;
	static KalmanState *state;
	static DelayedEstState gravity_est_state;
	static DelayedEstState ground_level_est_state;
	static DelayedEstState alt_est_state;
	float *accel = accel_get();
	float raw_alt = baro_get_altitude();
	// Only send telemetry every other step to avoid saturating the link and
	// increasing latency.  This variable can be temporarily overridden to
	// immediately send important updates (e.g., launch and apogee detection).
	static bool send_now = true;
	uint32_t step_time = millis();

	if (isnan(raw_alt) || isnan(accel[0])) {
		// Wait until the next run, by which time we may have new data.
		return;
	}

	kfloat_t accel_mag = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

	if (phase < FlightPhase::Launched) {
		calc_delayed_est(gravity_est_state, accel_mag);
		calc_delayed_est(ground_level_est_state, raw_alt);
	}

	if (phase == FlightPhase::Startup) {
		if (isnan(ground_level_est_state.old_old_est) ||
				isnan(gravity_est_state.old_old_est)) {
			return;
		}
		phase = FlightPhase::Idle;
	}

	accel_mag -= gravity_est_state.old_old_est;
	kfloat_t alt = raw_alt - ground_level_est_state.old_old_est;

	state = kalman_step(accel_mag, alt);

	if (phase == FlightPhase::Idle) {
		// Detect launch
		if (state->rate > LAUNCH_VELOCITY && state->accel > LAUNCH_ACCEL) {
			phase = FlightPhase::Launched;
			digitalWrite(PIN_LAUNCH, HIGH);
#if SD_LOG
			sd_commit(true);
#endif
			send_now = true;
		}
	} else if (phase == FlightPhase::Launched) {
		// Detect apogee
		if (state->rate < 0) {
			drogue_trigger_time = step_time;
			digitalWrite(PIN_DROGUE, HIGH);
			phase = FlightPhase::DescendingWithDrogue;

			Serial.println(F("===================================== Apogee!"));
#if SD_LOG
			Print *msgs = sd_messages();
			msgs->print("Drogue deployed at t=");
			msgs->print(step_time);
			msgs->println("us.");
#endif
			send_now = true;
		}
	} else if (phase == FlightPhase::DescendingWithDrogue) {
		// If we've reached apogee we won't be going very fast and
		// the barometer alone should be pretty reliable.

		// Deploy main if we are at the right altitude or if the drogue
		// fails to deploy and we're lawndarting.
		// Wait at least three seconds after drogue deployment though,
		// in case the pressure and acceleration of the deployment throws us off.
		if ((state->pos < MAIN_DEPLOY_ALTITUDE
#ifdef FAILSAFE_VELOCITY
			|| state->rate < -(FAILSAFE_VELOCITY)
#endif
				) && delta(drogue_trigger_time, step_time) > 3000) {
			phase = FlightPhase::DescendingWithMain;
			main_trigger_time = step_time;
			digitalWrite(PIN_MAIN, HIGH);

			Serial.println(F("===================================== Deploy main!"));
#if SD_LOG
			Print *msgs = sd_messages();
			msgs->print("Main deployed at t=");
			msgs->print(step_time);
			msgs->println("us.");
#endif
			send_now = true;
		}
	} else if (phase == FlightPhase::DescendingWithMain) {
		if (state->pos < LANDED_ALT &&
				abs(state->rate) < LANDED_VELOCITY &&
				abs(state->accel) < LANDED_ACCEL) {
			if (land_time == 0) {
				land_time = step_time;
				if (land_time == 0) land_time = 1;
			} else if (delta(land_time, step_time) > 5000) { // Must stay landed for 5 seconds
				phase = FlightPhase::Landed;
				Serial.println(F("===================================== Landed!"));
				send_now = true;
			}
		} else {
			land_time = 0;
		}
	} else if (phase == FlightPhase::Landed) {
#if SD_LOG
		sd_commit(false);
#endif
	}

	// Disable drogue igniter after 1 second
	if (phase >= FlightPhase::DescendingWithDrogue && delta(drogue_trigger_time, step_time) > 1000) {
		digitalWrite(PIN_DROGUE, LOW);
	}
	// Disable main igniter after 1 second
	if (phase >= FlightPhase::DescendingWithMain && delta(main_trigger_time, step_time) > 1000) {
		digitalWrite(PIN_MAIN, LOW);
	}

	uint16_t batt_v = (uint16_t)adc.analogReadContinuous(PIN_BATT_V);
	Serial.print("Raw batt V: ");
	Serial.println(batt_v);
	batt_v = map(batt_v, BATT_MIN_READING, BATT_FULL_READING, 0, BATT_FULL_VOLTAGE);

#if SD_LOG
	SdDataRecord dr;
	dr.time_ms = step_time;
	dr.temp = baro_get_temp();
	dr.state = *state;
	dr.altitude = raw_alt;
	dr.accel_x = accel[0];
	dr.accel_y = accel[1];
	dr.accel_z = accel[2];
	dr.lat = gps_get_lat();
	dr.lon = gps_get_lon();
	dr.gps_alt = gps_get_alt();
	dr.batt_v = batt_v;
	sd_log(dr);
#endif

	if (send_now) {
		radio_send(Packet(phase, step_time, state->pos, state->rate, state->accel,
				alt, accel_mag, gps_get_lat(), gps_get_lon(),
				baro_get_temp(), batt_v));
	}
	send_now = !send_now;
}
