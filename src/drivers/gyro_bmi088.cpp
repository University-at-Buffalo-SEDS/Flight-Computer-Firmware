#include "gyro.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <BMI088.h>

#include <cmath>

// Global function definitions
void gyro_setup();
void gyro_step();
float *gyro_get();
void gyro_print();

// File-Global variables
static float last_gyro[3] = {NAN, NAN, NAN};
Bmi088Gyro gyro(SPI, PIN_GYRO_CS);

// Global functions
void gyro_setup() {

    int status = gyro.begin();
    if (status < 0) {
        Serial.println("Gyro Init Error");
        Serial.println(status);
        abort();
    }

    status = gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_32HZ);

    if (!status) {
        Serial.println("Gyro Setup ODR Error");
        Serial.println(status);
        abort();
    }

    status = gyro.setRange(Bmi088Gyro::RANGE_2000DPS);

    if (!status) {
        Serial.println("Gyro Setup Range Error");
        Serial.println(status);
        abort();
    }

    scheduler_add(TaskId::Gyro, Task(gyro_step, KALMAN_PERIOD * 1000L, 120));
}

void gyro_step() {
    gyro.readSensor();

    last_gyro[0] = gyro.getGyroX_rads();
    last_gyro[1] = gyro.getGyroY_rads();
    last_gyro[2] = gyro.getGyroZ_rads();

}

float *gyro_get() {
    return last_gyro;
}

void gyro_print()
{
	float *a = gyro_get();
	Serial.print(F("Gyro: "));
	Serial.print(a[0]);
	Serial.print(F(", "));
	Serial.print(a[1]);
	Serial.print(F(", "));
	Serial.print(a[2]);
	Serial.println(F(" rad/s"));
}