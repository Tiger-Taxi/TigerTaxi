#ifndef ERROR_IND_H
#define ERROR_IND_H

#include "Arduino.h"

#define PIN_ERROR_LED1 47
#define PIN_ERROR_LED2 49
#define PIN_ERROR_SOUNDER 51

void setup_error_indication();

// Turn on or off error LED 1
void error_1 (bool on);

// Turn on or off error LED 2
void error_2 (bool on);

// Turn on or off error sounder
void error_sounder (bool on);

#endif
