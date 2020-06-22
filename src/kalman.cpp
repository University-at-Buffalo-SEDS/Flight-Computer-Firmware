// Based on http://home.earthlink.net/~david.schultz/rnd/2004/KalmanApogeeII.pdf

#include "config.hpp"
#include "kalman.hpp"
#include "util.hpp"

#ifndef NATIVE_TEST
#include <Arduino.h>
#endif

#include <cstdlib>

#include <cstdio>

// Computes the Kalman gain matrix
void KalmanFilter::calculate_gain(kfloat_t alt_sigma, kfloat_t accel_sigma, kfloat_t model_sigma)
{
	const kfloat_t alt_variance = alt_sigma * alt_sigma;
	const kfloat_t accel_variance = accel_sigma * accel_sigma;
	const kfloat_t model_variance = model_sigma * model_sigma;

	Matrix<kfloat_t, 3, 3> stm_t = stm.transposed();

	Matrix<kfloat_t, 3, 2> last_kgain = kgain;

	uint16_t iterations = 0;
	Matrix<kfloat_t, 3, 3> term;
	Matrix<kfloat_t, 3, 3> pest = {
		2, 0, 0,
		0, 9, 0,
		0, 0, 9
	};
	Matrix<kfloat_t, 3, 3> pestp = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
	};
	while (true) {
		// Propagate state covariance.
		term = stm * pest;

		pestp = term * stm_t;

		pestp(2, 2) = pestp(2, 2) + model_variance;

		// Calculate Kalman Gain
		kfloat_t det = (pestp(0, 0) + alt_variance) * (pestp(2, 2) + accel_variance) - pestp(2, 0) * pestp(0, 2);
		kgain(0, 0) = (pestp(0, 0) * (pestp(2, 2) + accel_variance) - pestp(0, 2) * pestp(2, 0)) / det;
		kgain(0, 1) = (pestp(0, 0) * (-pestp(0, 2)) + pestp(0, 2) * (pestp(0, 0) + alt_variance)) / det;
		kgain(1, 0) = (pestp(1, 0) * (pestp(2, 2) + accel_variance) - pestp(1, 2) * pestp(2, 0)) / det;
		kgain(1, 1) = (pestp(1, 0) * (-pestp(0, 2)) + pestp(1, 2) * (pestp(0, 0) + alt_variance)) / det;
		kgain(2, 0) = (pestp(2, 0) * (pestp(2, 2) + accel_variance) - pestp(2, 2) * pestp(2, 0)) / det;
		kgain(2, 1) = (pestp(2, 0) * (-pestp(0, 2)) + pestp(2, 2) * (pestp(0, 0) + alt_variance)) / det;

		pest(0, 0) = pestp(0, 0) * (1.0f - kgain(0, 0)) - kgain(0, 1) * pestp(2, 0);
		pest(0, 1) = pestp(0, 1) * (1.0f - kgain(0, 0)) - kgain(0, 1) * pestp(2, 1);
		pest(0, 2) = pestp(0, 2) * (1.0f - kgain(0, 0)) - kgain(0, 1) * pestp(2, 2);
		pest(1, 0) = pestp(0, 0) * (-kgain(1, 0)) + pestp(1, 0) - kgain(1, 1) * pestp(2, 0);
		pest(1, 1) = pestp(0, 1) * (-kgain(1, 0)) + pestp(1, 1) - kgain(1, 1) * pestp(2, 1);
		pest(1, 2) = pestp(0, 2) * (-kgain(1, 0)) + pestp(1, 2) - kgain(1, 1) * pestp(2, 2);
		pest(2, 0) = (1.0f - kgain(2, 1)) * pestp(2, 0) - kgain(2, 0) * pestp(2, 0);
		pest(2, 1) = (1.0f - kgain(2, 1)) * pestp(2, 1) - kgain(2, 0) * pestp(2, 1);
		pest(2, 2) = (1.0f - kgain(2, 1)) * pestp(2, 2) - kgain(2, 0) * pestp(2, 2);

		// Check for convergence. Criteria is less than .001% change from last time through the mill.
		++iterations;
		if ((kgain - last_kgain).norm_sq() / last_kgain.norm_sq() < 1e-12f) {
			break;
		}
		last_kgain = kgain;
	}

	DEBUG_SECTION(
		Serial.println(F("Input noise values used (standard deviation):"));
		Serial.print(F("Altitude       - "));
		Serial.print(sqrtf(alt_variance));
		Serial.println(F("m"));
		Serial.print(F("Acceleration   - "));
		Serial.print(sqrtf(accel_variance));
		Serial.println(F("m/s^2"));
		Serial.print(F("Model noise    - "));
		Serial.print(sqrtf(model_variance));
		Serial.println(F("m/s^2"));
		Serial.print(F("Kalman gains converged after "));
		Serial.print(iterations);
		Serial.println(F(" iterations:"));
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 2; j++) {
				Serial.print(kgain(i, j));
				if (i < 2 || j < 1)
					Serial.print(", ");
			}
		}
		Serial.println(F("\n"));
		Serial.println(F("Estimated output first order statistics (standard deviation):"));
		Serial.print(F("Altitude     - "));
		Serial.print(sqrtf(pest[0][0]));
		Serial.println(F("m"));
		Serial.print(F("Velocity     - "));
		Serial.print(sqrtf(pest[1][1]));
		Serial.println(F("m/s"));
		Serial.print(F("Acceleration - "));
		Serial.print(sqrtf(pest[2][2]));
		Serial.println(F("m/s^2"));
		// Output header for data.
		Serial.println(F("Time		Pressure	Acceleration	Est Pos		Est Rate	Est Accel"));
	)
}

void KalmanFilter::step(kfloat_t accel, kfloat_t altitude)
{
	if (first_step) {
		first_step = false;
		est(0) = altitude;
	}

	// Compute the innovations
	kfloat_t alt_inovation = altitude - estp(0);
	kfloat_t accel_inovation = accel - estp(2);

	// Code to ignore transsonic pressure effects.
	// First, check if we have a large error in altitude and are in the transonic range.
	if (abs(alt_inovation) > 30 && estp(1) > 300 && estp(1) < 400) {
		// We are going somewhere in the neighborhood of Mach 1 (360 m/s in the dry
		// New Mexico desert at 50C). Now check to see if we are slowing down.
		// Once the motor burns out we'll go into free-fall and the acceleration will
		// fall below gravitational acceleration.  After comphensating for local
		// gravitational acceleration, this means that the adjusted acceleration will
		// be negative, meaning that we're slowing down.
		if (estp(2) < 0) {
			// Assume that velocity and acceleration estimates are accurate.
			// Adjust current altitude estimate to be the same as the measured altitude.
			est(0) = altitude;
			alt_inovation = 0;
		}
	}

	// Simple check for over-range on pressure measurement. 
	if (altitude > 12000) {
		alt_inovation = 0;
	}

	// Propagate state
	estp = stm * est;

	// Update state
	est = estp + kgain * Vector<kfloat_t, 2>{alt_inovation, accel_inovation};

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
		Serial.print(est(0));
		Serial.print('\t');
		Serial.print(est(1));
		Serial.print('\t');
		Serial.println(est(2));
		last_run = micros();
	)
}
