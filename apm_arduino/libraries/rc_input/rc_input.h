#ifndef RC_H
#define RC_H

#include "Arduino.h"

// Pin for evelator which controls throttle and brake
static int PIN_EVELATOR_IN = 4;

// Pin for the aileron which controls steering
static int PIN_AILERON_IN = 3;

// Pin for the Rudder which controls the estop
static int PIN_ESTOP_IN = 2;

// Min and max pulse values
static int THROTTLE_MAX = 1290;
static int THROTTLE_MIN = 1080;
static int BRAKE_MAX = 1070;
static int BRAKE_MIN = 928;
static int STEERING_MIN = 1079;
static int STEERING_MAX = 1927;

// Structure to represent the rc_commands
struct RC_commands {
    double throttle;
    double brake;
    double steering;
    bool estop;
};

// Pulse-width timer variables
static int16_t elevator_pulse_width = 0;
static int16_t elevator_trig = 0;
static int16_t aileron_pulse_width = 0;
static int16_t aileron_trig = 0;
static int16_t estop_pulse_width = 0;
static int16_t estop_trig = 0;

///////////////
// Functions //
///////////////

// Setup function for the steering
void setup_rc();

// Reads and saves the present value of the steering
RC_commands get_rc_commands();

// Raw value to percent convertion functions
double _get_throttle_value (int elevator_raw);
double _get_brake_value (int elevator_raw);
double _get_steering_value (int aileron_raw);
bool _get_estop_value(int estop_pulse_width);

// ISRs for measuring pulse width of inputs
void _measure_pulse_elevator();
void _measure_pulse_aileron();
void _measure_pulse_estop();

#endif
