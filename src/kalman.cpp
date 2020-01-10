// Based on http://home.earthlink.net/~david.schultz/rnd/2004/KalmanApogeeII.pdf

#include <stdlib.h>
#include <Arduino.h>
#include "kalman.hpp"
#include "config.hpp"
#include "util.hpp"

// State transition matrix
static kfloat_t phi[3][3] = {
	1, KALMAN_PERIOD / 1000.0f, ((float)KALMAN_PERIOD * KALMAN_PERIOD) / 2000000.0f,
	0, 1, KALMAN_PERIOD / 1000.0f,
	0, 0, 1};

#ifdef KALMAN_GAINS

static kfloat_t kgain[3][2] = {KALMAN_GAINS};
void kalman_setup() {}

#else

static kfloat_t kgain[3][2] = {0, 0, 0, 0, 0, 0};

#define ALTITUDE_VARIANCE (ALTITUDE_SIGMA * ALTITUDE_SIGMA)
#define ACCELERATION_VARIANCE (ACCELERATION_SIGMA * ACCELERATION_SIGMA)
#define MODEL_VARIANCE (MODEL_SIGMA * MODEL_SIGMA)

// Calculates Kalman gains
void kalman_setup()
{
	kfloat_t dt = KALMAN_PERIOD / 1000.0f;
	// Fill in state transition matrix and its transpose.
	// Transpose of phi
	static kfloat_t phit[3][3] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1};
	phi[0][1] = dt;
	phi[1][2] = dt;
	phi[0][2] = dt * dt / 2.0f;
	phit[1][0] = dt;
	phit[2][1] = dt;
	phit[2][0] = dt * dt / 2.0f;
	// Compute the Kalman gain matrix.
	static kfloat_t lastkgain[3][2];
	for (uint8_t i = 0; i < 3; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			lastkgain[i][j] = kgain[i][j];
		}
	}
	uint16_t iterations = 0;
	kfloat_t term[3][3];
	kfloat_t pest[3][3] = {
		2, 0, 0,
		0, 9, 0,
		0, 0, 9};
	kfloat_t pestp[3][3] = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0};
	while (true) {
		// Propagate state covariance.
		term[0][0] = phi[0][0] * pest[0][0] + phi[0][1] * pest[1][0] + phi[0][2] * pest[2][0];
		term[0][1] = phi[0][0] * pest[0][1] + phi[0][1] * pest[1][1] + phi[0][2] * pest[2][1];
		term[0][2] = phi[0][0] * pest[0][2] + phi[0][1] * pest[1][2] + phi[0][2] * pest[2][2];
		term[1][0] = phi[1][0] * pest[0][0] + phi[1][1] * pest[1][0] + phi[1][2] * pest[2][0];
		term[1][1] = phi[1][0] * pest[0][1] + phi[1][1] * pest[1][1] + phi[1][2] * pest[2][1];
		term[1][2] = phi[1][0] * pest[0][2] + phi[1][1] * pest[1][2] + phi[1][2] * pest[2][2];
		term[2][0] = phi[2][0] * pest[0][0] + phi[2][1] * pest[1][0] + phi[2][2] * pest[2][0];
		term[2][1] = phi[2][0] * pest[0][1] + phi[2][1] * pest[1][1] + phi[2][2] * pest[2][1];
		term[2][2] = phi[2][0] * pest[0][2] + phi[2][1] * pest[1][2] + phi[2][2] * pest[2][2];
		pestp[0][0] = term[0][0] * phit[0][0] + term[0][1] * phit[1][0] + term[0][2] * phit[2][0];
		pestp[0][1] = term[0][0] * phit[0][1] + term[0][1] * phit[1][1] + term[0][2] * phit[2][1];
		pestp[0][2] = term[0][0] * phit[0][2] + term[0][1] * phit[1][2] + term[0][2] * phit[2][2];
		pestp[1][0] = term[1][0] * phit[0][0] + term[1][1] * phit[1][0] + term[1][2] * phit[2][0];
		pestp[1][1] = term[1][0] * phit[0][1] + term[1][1] * phit[1][1] + term[1][2] * phit[2][1];
		pestp[1][2] = term[1][0] * phit[0][2] + term[1][1] * phit[1][2] + term[1][2] * phit[2][2];
		pestp[2][0] = term[2][0] * phit[0][0] + term[2][1] * phit[1][0] + term[2][2] * phit[2][0];
		pestp[2][1] = term[2][0] * phit[0][1] + term[2][1] * phit[1][1] + term[2][2] * phit[2][1];
		pestp[2][2] = term[2][0] * phit[0][2] + term[2][1] * phit[1][2] + term[2][2] * phit[2][2];
		pestp[2][2] = pestp[2][2] + MODEL_VARIANCE;
		// Calculate Kalman Gain
		kfloat_t det = (pestp[0][0] + ALTITUDE_VARIANCE) * (pestp[2][2] + ACCELERATION_VARIANCE) - pestp[2][0] * pestp[0][2];
		kgain[0][0] = (pestp[0][0] * (pestp[2][2] + ACCELERATION_VARIANCE) - pestp[0][2] * pestp[2][0]) / det;
		kgain[0][1] = (pestp[0][0] * (-pestp[0][2]) + pestp[0][2] * (pestp[0][0] + ALTITUDE_VARIANCE)) / det;
		kgain[1][0] = (pestp[1][0] * (pestp[2][2] + ACCELERATION_VARIANCE) - pestp[1][2] * pestp[2][0]) / det;
		kgain[1][1] = (pestp[1][0] * (-pestp[0][2]) + pestp[1][2] * (pestp[0][0] + ALTITUDE_VARIANCE)) / det;
		kgain[2][0] = (pestp[2][0] * (pestp[2][2] + ACCELERATION_VARIANCE) - pestp[2][2] * pestp[2][0]) / det;
		kgain[2][1] = (pestp[2][0] * (-pestp[0][2]) + pestp[2][2] * (pestp[0][0] + ALTITUDE_VARIANCE)) / det;
		pest[0][0] = pestp[0][0] * (1.0f - kgain[0][0]) - kgain[0][1] * pestp[2][0];
		pest[0][1] = pestp[0][1] * (1.0f - kgain[0][0]) - kgain[0][1] * pestp[2][1];
		pest[0][2] = pestp[0][2] * (1.0f - kgain[0][0]) - kgain[0][1] * pestp[2][2];
		pest[1][0] = pestp[0][0] * (-kgain[1][0]) + pestp[1][0] - kgain[1][1] * pestp[2][0];
		pest[1][1] = pestp[0][1] * (-kgain[1][0]) + pestp[1][1] - kgain[1][1] * pestp[2][1];
		pest[1][2] = pestp[0][2] * (-kgain[1][0]) + pestp[1][2] - kgain[1][1] * pestp[2][2];
		pest[2][0] = (1.0f - kgain[2][1]) * pestp[2][0] - kgain[2][0] * pestp[2][0];
		pest[2][1] = (1.0f - kgain[2][1]) * pestp[2][1] - kgain[2][0] * pestp[2][1];
		pest[2][2] = (1.0f - kgain[2][1]) * pestp[2][2] - kgain[2][0] * pestp[2][2];
		// Check for convergance. Criteria is less than .001% change from last time through the mill.
		uint8_t not_done = 0;
		++iterations;
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 2; j++) {
				if ((kgain[i][j] - lastkgain[i][j])/lastkgain[i][j] > 0.00001f) {
					not_done++;
				}
				lastkgain[i][j] = kgain[i][j];
			}
		}
		if (not_done > 0) {
			continue;
		} else {
			break;
		}
	}

	DEBUG_SECTION(
		Serial.println(F("Input noise values used (standard deviation):"));
		Serial.print(F("Altitude       - "));
		Serial.print(sqrt(ALTITUDE_VARIANCE));
		Serial.println(F("m"));
		Serial.print(F("Acceleration   - "));
		Serial.print(sqrt(ACCELERATION_VARIANCE));
		Serial.println(F("m/s^2"));
		Serial.print(F("Model noise    - "));
		Serial.print(sqrt(MODEL_VARIANCE));
		Serial.println(F("m/s^2"));
		Serial.print(F("Kalman gains converged after "));
		Serial.print(iterations);
		Serial.println(F(" iterations:"));
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 2; j++) {
				Serial.print(kgain[i][j]);
				if (i < 2 || j < 1)
					Serial.print(", ");
			}
		}
		Serial.println(F("\n"));
		Serial.println(F("Estimated output first order statistics (standard deviation):"));
		Serial.print(F("Altitude     - "));
		Serial.print(sqrt(pest[0][0]));
		Serial.println(F("m"));
		Serial.print(F("Velocity     - "));
		Serial.print(sqrt(pest[1][1]));
		Serial.println(F("m/s"));
		Serial.print(F("Acceleration - "));
		Serial.print(sqrt(pest[2][2]));
		Serial.println(F("m/s^2"));
		// Output header for data.
		Serial.println(F("Time		Pressure	Acceleration	Est Pos		Est Rate	Est Accel"));
	)
}

#endif

KalmanState *kalman_step(kfloat_t accel, kfloat_t altitude)
{
	// Estimation prediction
	static KalmanState estp = {0, 0, 0};
	// Current estimation
	static KalmanState est = {0, 0, 0};
	static bool init_done = false;

	if (!init_done) {
		init_done = true;
		est.pos = altitude;
	}

	// Compute the innovations
	kfloat_t alt_inovation = altitude - estp.pos;
	kfloat_t accel_inovation = accel - estp.accel;

	// Code to ignore transsonic pressure effects.
	// First, check if we have a large error in altitude and are in the transonic range.
	if (abs(alt_inovation) > 30 && estp.rate > 300 && estp.rate < 400) {
		// We are going somewhere in the neighborhood of Mach 1 (360 m/s in the dry
		// New Mexico desert at 50C). Now check to see if we are slowing down.
		// Once the motor burns out we'll go into free-fall and the acceleration will
		// fall below gravitational acceleration.  After comphensating for local
		// gravitational acceleration, this means that the adjusted acceleration will
		// be negative, meaning that we're slowing down.
		if (estp.accel < 0) {
			// Assume that velocity and acceleration estimates are accurate.
			// Adjust current altitude estimate to be the same as the measured altitude.
			est.pos = altitude;
			alt_inovation = 0;
		}
	}

	// Simple check for over-range on pressure measurement. 
	if (altitude > 12000) {
		alt_inovation = 0;
	}

	// Propagate state
	// estp = phi * est;
	estp.pos   = phi[0][0] * est.pos + phi[0][1] * est.rate + phi[0][2] * est.accel;
	estp.rate  = phi[1][0] * est.pos + phi[1][1] * est.rate + phi[1][2] * est.accel;
	estp.accel = phi[2][0] * est.pos + phi[2][1] * est.rate + phi[2][2] * est.accel;

	// Update state
	// est = estp + kgain * (z - x);
	est.pos   = estp.pos   + kgain[0][0] * alt_inovation + kgain[0][1] * accel_inovation;
	est.rate  = estp.rate  + kgain[1][0] * alt_inovation + kgain[1][1] * accel_inovation;
	est.accel = estp.accel + kgain[2][0] * alt_inovation + kgain[2][1] * accel_inovation;

	// Output
	DEBUG_SECTION(
		static uint32_t last_run = micros();
		Serial.print(millis());
		Serial.print('\t');
		Serial.print(sdelta(last_run, micros()));
		Serial.print('\t');
		Serial.print(altitude);
		Serial.print('\t');
		Serial.print(accel);
		Serial.print('\t');
		Serial.print(est.pos);
		Serial.print('\t');
		Serial.print(est.rate);
		Serial.print('\t');
		Serial.println(est.accel);
		last_run = micros();
	)

	return &est;
}
