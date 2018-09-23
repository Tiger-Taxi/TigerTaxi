#ifndef CART_ODOM_H
#define CART_ODOM_H

#include "Arduino.h"
#include "rosdue.h"
#include "steering.h"
#include "pwm_lib.h"

#define PIN_VTACH 6

// How often ODOM is evaluated (in millis)
#define ODOM_PERIOD 10000

// The distance traveled in one tick
#define VTACH_TICK_DISTANCE (0.026191 / 2)

// Threshold for counting a period as noise and not sending it
#define VALID_SPEED_THRESHOLD 2.32

//
#define STEERING_ANGLE_FACTOR 0.026

// Sets up the cart odometry
void setup_cart_odom(ROSdue ros);

// Called in main loop every time
void cart_odom_spin();

// Returns the current speed of the cart
float get_cart_speed();

// Convert from a desired steering percent to a desired steering angle
float get_steering_percent_to_angle(float percent);

// ISR for the vtack
void _vtach_tick();

#endif
