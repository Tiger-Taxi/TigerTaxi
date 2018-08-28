#ifndef STEERING_H
#define STEERING_H

#include "Arduino.h"
#include "RunningAverage.h"
#include "PID_v1.h"
#include "cart_odom.h"

/* Pin config */
#define PIN_STEERING_POT A0
#define PIN_STEERING_OUT DAC0

/* Min and Max values for the potentiometer */
#define STEERING_POT_MIN 1580 // Full Left
#define STEERING_POT_MAX 3794 // Full Right
#define STEERING_POT_CENTER ((STEERING_POT_MAX + STEERING_POT_MIN)/2)

// Slope of the line to convert position 0-100 to a pot value
#define STEERING_M ((STEERING_POT_MAX - STEERING_POT_MIN) / 100.0)

/* Values to write to the DAC */
#define STEER_CENTER 2075            // 'Center' (Vdiff = 0) value for the differential output Old: 2117
#define STEERING_CORRECTION_MAX 1500 // Max value we can add or subtract to center

#define STEERING_PID_SAMPLE_TIME 1 // Sample time in ms that the PID is evaluated at

/* Averager for the steering pot */
static RunningAverage steering_averager(1);

/* PID */
#define Kp 1.5 // Proportional tuning param
#define Ki 0 // Integral tuning param
#define Kd 0 // Derivative tuning param

static double current_steering_pot_value = 0; // Input to PID
static double desired_steering_pot_value = 0; // Desired value for PID
static double steering_pid_output = 0; // Output of the PID system

// PID controller object
static PID steering_PID(&current_steering_pot_value, &steering_pid_output, &desired_steering_pot_value, Kp, Ki, Kd, DIRECT);

///////////////
// Functions //
///////////////

// Setup function for the steering
void setup_steering();

// Reads, saves, and returns the present value of the steering
int get_position_steering();

// Sets up the PID system
void setup_PID();

// Sets the steering rack to a potentiometer value
void set_steering_pot_value(double pot_val);

// Set the desired position for the steering,
// Where from driver's seat 0 is full left, 100 is full right, and 50 is in the middle
void set_steering_percentage(double steering_position);

// Set the desired steering angle
// If the angle is outside of the range it can go to, clips to edge
void set_steering_angle(float steering_angle);

// Sends the steering rack to the left
void steer_left();

// Sends the steering rack to the right
void steer_right();

// Sends the steering rack a signal that does nothing
void stop_steering();

#endif
