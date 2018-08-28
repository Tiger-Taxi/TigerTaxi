#ifndef THROTTLE_H
#define THROTTLE_H

#include "Arduino.h"

// Config //
const int PIN_THROTTLE_IN = A10;
const int PIN_THROTTLE_OUT = DAC1;

// The current throttle percentage to output
static double throttle_output;

// The current value of the throttle sensor (pedal)
static int throttle_sensor_value;

///////////////
// Functions //
///////////////

// Setup function for the throttle
void setup_throttle();

// Reads and saves the present value of the throttle
double get_position_throttle();

// Set the desired throttle percentage
void set_throttle(double throttle);

#endif
