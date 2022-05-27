#include "accel.hpp"
#include "baro.hpp"
#include "config.hpp"
#include "gps.hpp"
#include "kalman.hpp"
#include "log.hpp"
#include "radio.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <SPI.h>
#include <Wire.h>

#include <cmath>

struct ChannelStatus {
	uint32_t fire_time;
	bool firing;
};

// Prototypes
void command_step();
void blink_step();
void print_step();
void deployment_step();
void channel_step();
void channel_fire(Channel chan);
void rgb_step();
void rgb_color(int r, int g, int b);

static std::array<ChannelStatus, channel_config.size()> channel_status;

#ifdef KALMAN_GAINS
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f, {KALMAN_GAINS});
#else
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f,
		ALTITUDE_SIGMA, ACCELERATION_SIGMA, MODEL_SIGMA);
#endif

void setup()
{
	for (const ChannelConfig &c : channel_config) {
		pinMode(c.fire_pin, OUTPUT);
		digitalWrite(c.fire_pin, LOW);
	}

#ifdef PIN_LAUNCH
	pinMode(PIN_LAUNCH, OUTPUT);
	digitalWrite(PIN_LAUNCH, LOW);
#endif

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	// Setup RGB leds
	pinMode(LED_RED, OUTPUT);
	digitalWrite(LED_RED, LOW);

	pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, LOW);

	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, LOW);

	// Setup Buzzer
	pinMode(PIN_BUZZER, OUTPUT);
	digitalWrite(PIN_BUZZER, LOW);

	// pinMode(PIN_BATT_V, INPUT_ANALOG);
	// pinMode(PIN_SYS_V, INPUT_ANALOG);
	analogReadResolution(12);  // Enable full resolution

	Serial.begin(9'600);

	Serial.println(F("Flight Computer " __DATE__ " " __TIME__));

	Wire.begin();

	SPI.begin();

	gps_setup();
	baro_setup();
	accel_setup();
#if LOG_ENABLE
	log_setup();
#endif
	radio_setup();

	scheduler_add(TaskId::Deployment, Task(deployment_step, KALMAN_PERIOD * 1000L, 2500));
	scheduler_add(TaskId::ChannelTimeout, Task(channel_step,
			CHANNEL_FIRE_TIME * 100L, 10));
	scheduler_add(TaskId::Command, Task(command_step, 100'000L, 10));
	scheduler_add(TaskId::Print, Task(print_step, 3'000'000L, 3000));
	scheduler_add(TaskId::Blink, Task(blink_step, (KALMAN_PERIOD / 2) * 1000L, 20));
	rgb_color(1, 0, 0);
}

void loop()
{
	uint32_t wait_time = schedule();
	if (wait_time > 4) {
		delayMicroseconds(wait_time - 4);
	}
}

void channel_fire(Channel chan)
{
	ChannelStatus &status = channel_status[(size_t)chan];
	status.firing = true;
	status.fire_time = millis();
	digitalWrite(channel_config[(size_t)chan].fire_pin, HIGH);
}

void channel_step()
{
	uint32_t now = millis();
	for (size_t i = 0; i < channel_status.size(); ++i) {
		ChannelStatus &status = channel_status[i];
		const ChannelConfig &config = channel_config[i];
		if (status.firing && delta(status.fire_time, now) > CHANNEL_FIRE_TIME) {
			status.firing = false;
			digitalWrite(config.fire_pin, LOW);
		}
	}
}

void command_step()
{
	switch (Serial.read()) {
	case 'r':
		log_print_all();
		break;
	default:
		Serial.println("Unrecognized command.");
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
	gps_print();
	accel_print();
	baro_print();
}

void rgb_color(int r, int g, int b)
{
	digitalWrite(LED_RED, r ? HIGH : LOW);
	digitalWrite(LED_GREEN, g ? HIGH : LOW);
	digitalWrite(LED_BLUE, b ? HIGH : LOW);
}

void deployment_step()
{
	static uint32_t land_time = 0;
	static FlightPhase phase = FlightPhase::Startup;
	static AvgHistory<float, EST_HISTORY_SAMPLES, 3> gravity_est_state;
	static AvgHistory<float, EST_HISTORY_SAMPLES, 3> ground_level_est_state;
	static kfloat_t apogee = 0;
	float *accel = accel_get();
	float raw_alt = baro_get_altitude();
	// Only send telemetry every other step to avoid saturating the link and
	// increasing latency.  This variable can be temporarily overridden to
	// immediately send important updates (e.g., launch and apogee detection).
	static bool send_now = true;
	uint32_t step_time = millis();

	if (std::isnan(raw_alt) || std::isnan(accel[0])) {
		// Wait until the next run, by which time we may have new data.
		return;
	}

	float accel_mag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

	if (phase < FlightPhase::Launched) {
		gravity_est_state.add(accel_mag);
		ground_level_est_state.add(raw_alt);
	}

	if (phase == FlightPhase::Startup) {
		if (!ground_level_est_state.full() ||
				!gravity_est_state.full()) {
			return;
		}
		rgb_color(1, 1, 0);
		phase = FlightPhase::Idle;
	}

	accel_mag -= gravity_est_state.old_avg();
	float alt = raw_alt - ground_level_est_state.old_avg();

	bool any_channel_firing = false;
	for (const ChannelStatus &s : channel_status) {
		if (s.firing) {
			any_channel_firing = true;
			break;
		}
	}

	// Skip kalman filter shortly after deployment to ignore
	// the corresponding pressure/acceleration spikes.
	if (!any_channel_firing) {
		kf.step(accel_mag, alt);
	}

	if (phase == FlightPhase::Idle) {
		// Detect launch
		if (kf.rate() > LAUNCH_VELOCITY && kf.accel() > LAUNCH_ACCEL) {
			rgb_color(0, 1, 0);
			phase = FlightPhase::Launched;
#ifdef PIN_LAUNCH
			digitalWrite(PIN_LAUNCH, HIGH);
#endif
#if LOG_ENABLE
			// log_start();
#endif
			send_now = true;
		}
	} else if (phase == FlightPhase::Launched) {
		// Detect apogee
		if (kf.rate() < 0) {
			apogee = kf.pos();
			channel_fire(Channel::Drogue);
			rgb_color(0, 1, 1);
			phase = FlightPhase::DescendingWithDrogue;

			Serial.println(F("===================================== Apogee!"));
			send_now = true;
		}
	} else if (phase == FlightPhase::DescendingWithDrogue) {
		// If we've reached apogee we won't be going very fast and
		// the barometer alone should be pretty reliable.

		// Deploy main if we are at the right altitude or if the drogue
		// fails to deploy and we're lawndarting.
		// Wait at least three seconds after drogue deployment though,
		// in case the pressure and acceleration of the deployment throws us off.
		if ((kf.pos() < MAIN_DEPLOY_ALTITUDE
#ifdef FAILSAFE_VELOCITY
			|| kf.rate() < -(FAILSAFE_VELOCITY)
#endif
				) && delta(channel_status[(size_t)Channel::Drogue].fire_time, step_time) > 3000) {
			rgb_color(0, 0, 1);
			phase = FlightPhase::DescendingWithMain;
			channel_fire(Channel::Main);

			Serial.println(F("===================================== Deploy main!"));
			send_now = true;
		}
	} else if (phase == FlightPhase::DescendingWithMain) {
		if (kf.pos() < LANDED_ALT &&
				abs(kf.rate()) < LANDED_VELOCITY &&
				abs(kf.accel()) < LANDED_ACCEL) {
			if (land_time == 0) {
				land_time = step_time;
				if (land_time == 0) {
					land_time = 1;
				}
			} else if (delta(land_time, step_time) > LANDED_TIME_MS) { // Must stay landed long enough
				rgb_color(1, 0, 1);
				phase = FlightPhase::Landed;
				Serial.println(F("===================================== Landed!"));
				send_now = true;
#if LOG_ENABLE
				log_stop();
#endif
			}
		} else {
			land_time = 0;
		}
	}

	uint32_t batt_v = 0;
	// uint32_t batt_v = analogRead(PIN_BATT_V);
	// Serial.print("Raw batt V: ");
	// Serial.println(batt_v);
	// batt_v = map(batt_v, BATT_MIN_READING, BATT_MAX_READING, 0, BATT_MAX_VOLTAGE);

	// uint32_t sys_v = analogRead(PIN_SYS_V);
	// sys_v = map(sys_v, SYS_MIN_READING, SYS_MAX_READING, 0, SYS_MAX_VOLTAGE);

#if LOG_ENABLE
	log_add(LogMessage(
		step_time,
		kf.state(),
		raw_alt,
		accel[0],
		accel[1],
		accel[2],
		gps_get_lat(),
		gps_get_lon(),
		gps_get_alt(),
		baro_get_temp(),
		(uint16_t)batt_v,
		(uint16_t)sys_v
	));
#endif

	if (send_now) {
		radio_send(Packet(phase, step_time, kf.pos(), kf.rate(), kf.accel(),
				alt, accel_mag, gps_get_lat(), gps_get_lon(), apogee,
				baro_get_temp(), batt_v));
	}

	send_now = !send_now;
}
