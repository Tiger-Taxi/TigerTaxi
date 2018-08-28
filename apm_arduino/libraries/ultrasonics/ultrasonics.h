#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "Arduino.h"
#include "RunningMedian.h"

#define PIN_U1 A5
#define PIN_U2 A6
#define PIN_U3 A7
#define PIN_U4 A8
#define PIN_U5 A9
#define PIN_U6 A10
#define PIN_U7 A11

#define PIN_ULTRASONIC_TRIGGER 23

// Correction factor to convert distance to inches
#define ULTRASONIC_CORRECTION_FACTOR_7001 0.0068027
#define ULTRASONIC_CORRECTION_FACTOR_7363 0.0393701

// Conversion factor to meters
#define INCHES_TO_METERS 0.0254

// Sets up ultrasonics
void setup_ultrasonics();

// Sets Ultrasonic Trigger pin
void read_ultrasonics();

// ISR for Ultrasonics
void _ultrasonic_1_change();
void _ultrasonic_2_change();
void _ultrasonic_3_change();
void _ultrasonic_4_change();
void _ultrasonic_5_change();
void _ultrasonic_6_change();
void _ultrasonic_7_change();

// Gets the distance values from the ultrasonics
float get_ultrasonic_1();
float get_ultrasonic_2();
float get_ultrasonic_3();
float get_ultrasonic_4();
float get_ultrasonic_5();
float get_ultrasonic_6();
float get_ultrasonic_7();

/*

// Gets the distance values from the ultrasonics
long get_ultrasonic_x1();
long get_ultrasonic_x2();
long get_ultrasonic_x3();
long get_ultrasonic_x4();
long get_ultrasonic_x5();
long get_ultrasonic_x6();
long get_ultrasonic_x7();

// Gets the distance values from the ultrasonics
long get_ultrasonic_y1();
long get_ultrasonic_y2();
long get_ultrasonic_y3();
long get_ultrasonic_y4();
long get_ultrasonic_y5();
long get_ultrasonic_y6();
long get_ultrasonic_y7();

*/

#endif
