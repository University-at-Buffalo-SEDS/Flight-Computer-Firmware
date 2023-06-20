#include <Arduino.h>
#include "config.hpp"
#include "buzzer.hpp"

void buzzer_on();
void buzzer_ready();

void buzzer_beep(uint16_t on_time_ms, uint16_t off_time_ms) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(on_time_ms);
    digitalWrite(PIN_BUZZER, LOW);
    delay(off_time_ms);
}

void buzzer_on() {
    buzzer_beep(1000, 250);
}

void buzzer_ready() {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            buzzer_beep(250, 250);
        }

        delay(750);
    }
}