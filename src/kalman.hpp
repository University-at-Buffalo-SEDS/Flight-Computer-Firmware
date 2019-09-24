#pragma once

typedef float kfloat_t;

struct KalmanState {
	kfloat_t pos, rate, accel;
};

void kalman_setup();

// Accel in m/s^2, altitude in m (calculated based on pressure).
// Constant time step determined beforehand!
KalmanState *kalman_step(kfloat_t accel, kfloat_t altitude);
