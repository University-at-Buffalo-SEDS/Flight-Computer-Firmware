#include "accel.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <BMI088.h>

#include <cmath>

// Global function definitions
void accel_setup();
void accel_step();
float *accel_get();
void accel_print();

// File-Global variables
static float last_accel[3] = {NAN, NAN, NAN};
Bmi088Accel accel(SPI, PIN_BMI088_Accel_CS);

// Global functions
void accel_setup() {

    int status = accel.begin();
    if (status < 0) {
        Serial.println("Accel Init Error");
        Serial.println(status);
        abort();
    }

    status = accel.setOdr(Bmi088Accel::ODR_100HZ_BW_40HZ);

    if (status) {
        Serial.println("Accel Setup ODR Error");
        Serial.println(status);
        abort();
    }

    status = accel.setRange(Bmi088Accel::RANGE_12G);

    if (status) {
        Serial.println("Accel Setup Range Error");
        Serial.println(status);
        abort();
    }

    scheduler_add(TaskId::Accel, Task(accel_step, KALMAN_PERIOD * 1000L, 120));
}

void accel_step() {
    accel.readSensor();

    last_accel[0] = accel.getAccelX_mss();
    last_accel[1] = accel.getAccelY_mss();
    last_accel[2] = accel.getAccelZ_mss();

}

float *accel_get() {
    return last_accel;
}

void accel_print()
{
	float *a = accel_get();
	Serial.print(F("Accel: "));
	Serial.print(a[0]);
	Serial.print(F(", "));
	Serial.print(a[1]);
	Serial.print(F(", "));
	Serial.print(a[2]);
	Serial.print(F(" ("));
	Serial.print(sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
	Serial.println(F(") m/s^2"));
}