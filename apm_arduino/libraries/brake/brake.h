#ifndef BRAKE_H
#define BRAKE_H

#include <Servo.h>
#include "Arduino.h"
#include "RunningAverage.h"

// Pin Config
const int PIN_BRAKE_POT_IN = A2;
const int PIN_BRAKE_OUT = 7;

// Object used to control the brake actuator
static Servo brake_actuator;

// min and max pulse-width for the brake_actuator in microseconds
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;

// Brake write values for brake_actuator
const int BRAKE_RETRACT = 180;
const int BRAKE_STAY = 10;
const int BRAKE_EXTEND = 0;

// Brake Pot position min and max values
const int BRAKE_POS_MAX = 480;
const int BRAKE_POS_MIN = 0;

// Desired brake amount (percentage)
static double desired_brake_amount;

// The current value from the brake pot
static int current_brake_position;

// Running Average of the current brake position
static RunningAverage brake_averager(10);

//////////////////////////////////
// Brake Damping control system //
//////////////////////////////////

// Number of cycles we have to delay
static int delay_cycles = 0;

// Number of times we should delay
static int delay_amount = 0;

// The time (in ms) since the last brake command
static unsigned long time_since_last_command = millis();

// Damping Region 0 (max damping)
static const int DAMPING_REGION_0_DELTA = 50; // Where this region ends (pot value) //50
static const int DAMPING_REGION_0_DELAY = 30; // How many cycles to delay //30

// Damping Region 1 (moderate damping)
static const int DAMPING_REGION_1_DELTA = 100; // 100
static const int DAMPING_REGION_1_DELAY = 12; // 12

///////////////
// Functions //
///////////////

// Setup function for the brake
void setup_brake();

// Gets the current position of the brake pot.
// This must be done before set_brake, split out for timing optimization
int get_position_brake();

void brake_extend();

void brake_retract();

void brake_stay();

// Set and save the desired position for the brake
void set_brake(double brake_amount);

#endif
