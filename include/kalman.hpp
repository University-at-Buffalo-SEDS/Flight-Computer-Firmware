#pragma once
#include "matrix.hpp"

using kfloat_t = float;

using KalmanState = Vector<kfloat_t, 3>;

class KalmanFilter {
	// State transition matrix
	Matrix<kfloat_t, 3, 3> stm;

	// Kalman gain matrix
	Matrix<kfloat_t, 3, 2> kgain;

	// Estimate prediction
	KalmanState estp;

	// Current estimate
	KalmanState est;

	bool first_step;

	KalmanFilter(kfloat_t time_step) :
		stm {
			1, time_step, time_step * time_step / 2.0f,
			0, 1, time_step,
			0, 0, 1
		},
		first_step(true)
	{}

	void calculate_gain(kfloat_t alt_sigma, kfloat_t accel_sigma, kfloat_t model_sigma);

public:
	// Construct with generated gain matrix
	KalmanFilter(float time_step, kfloat_t alt_sigma, kfloat_t accel_sigma, kfloat_t model_sigma) :
			KalmanFilter(time_step)
	{
		calculate_gain(alt_sigma, accel_sigma, model_sigma);
	}

	// Construct with pre-computed gain matrix
	KalmanFilter(float time_step, const Matrix<kfloat_t, 3, 2> &gain) :
			KalmanFilter(time_step)
	{
		kgain = gain;
	}

	// Accel in m/s^2, altitude in m (calculated based on pressure).
	// Constant time step determined beforehand!
	void step(kfloat_t accel, kfloat_t altitude);

	KalmanState &state() { return est; }
	kfloat_t pos() const { return est(0); }
	kfloat_t rate() const { return est(1); }
	kfloat_t accel() const { return est(2); }
};
