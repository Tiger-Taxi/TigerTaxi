#include "error_indication.h"

// Sets up error indication
void setup_error_indication() {
    pinMode(PIN_ERROR_LED1, OUTPUT);
    pinMode(PIN_ERROR_LED2, OUTPUT);
    pinMode(PIN_ERROR_SOUNDER, OUTPUT);
}

// Turn on or off error LED 1
void error_1 (bool on) {
    if (on) {
        digitalWrite(PIN_ERROR_LED1, HIGH);
    } else {
        digitalWrite(PIN_ERROR_LED1, LOW);
    }
}

// Turn on or off error LED 2
void error_2 (bool on) {
    if (on) {
        digitalWrite(PIN_ERROR_LED2, HIGH);
    } else {
        digitalWrite(PIN_ERROR_LED2, LOW);
    }
}

// Turn on or off error sounder
void error_sounder (bool on) {
    if (on) {
        digitalWrite(PIN_ERROR_SOUNDER, LOW);
    } else {
        digitalWrite(PIN_ERROR_SOUNDER, HIGH);
    }
}
