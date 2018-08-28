#include "ultrasonics.h"

/*

// Variables that store ultrasonic y distance
volatile long ultrasonic_1_xdistance,
    ultrasonic_2_xdistance,
    ultrasonic_3_xdistance,
    ultrasonic_4_xdistance,
    ultrasonic_5_xdistance,
    ultrasonic_6_xdistance,
    ultrasonic_7_xdistance;

// Variables that store ultrasonic x distance
volatile long ultrasonic_1_ydistance,
    ultrasonic_2_ydistance,
    ultrasonic_3_ydistance,
    ultrasonic_4_ydistance,
    ultrasonic_5_ydistance,
    ultrasonic_6_ydistance,
    ultrasonic_7_ydistance;

*/

// Median filters for sensor pulse widths
RunningMedian _filter_ultrasonic_1 = RunningMedian(3);
RunningMedian _filter_ultrasonic_2 = RunningMedian(3);
RunningMedian _filter_ultrasonic_3 = RunningMedian(3);
RunningMedian _filter_ultrasonic_4 = RunningMedian(3);
RunningMedian _filter_ultrasonic_5 = RunningMedian(3);
RunningMedian _filter_ultrasonic_6 = RunningMedian(3);
RunningMedian _filter_ultrasonic_7 = RunningMedian(3);

// Variables that store the time of the rising edge from ultrasonic
volatile long ultrasonic_1_time_rising,
    ultrasonic_2_time_rising,
    ultrasonic_3_time_rising,
    ultrasonic_4_time_rising,
    ultrasonic_5_time_rising,
    ultrasonic_6_time_rising,
    ultrasonic_7_time_rising;

/*
volatile int u1_theta = 45,
    u2_theta = 15,
    u6_theta = 15,
    u7_theta = 45;
*/

// Sets up ultrasonics
void setup_ultrasonics() {

    // Sets trigger pin to output
    pinMode(PIN_ULTRASONIC_TRIGGER, OUTPUT);

    // Sets Ultrasonic pins to digital input pins
    pinMode(PIN_U1, INPUT);
    digitalWrite(PIN_U1, HIGH);
    pinMode(PIN_U2, INPUT);
    digitalWrite(PIN_U2, HIGH);
    pinMode(PIN_U3, INPUT);
    digitalWrite(PIN_U3, HIGH);
    pinMode(PIN_U4, INPUT);
    digitalWrite(PIN_U4, HIGH);
    pinMode(PIN_U5, INPUT);
    digitalWrite(PIN_U5, HIGH);
    pinMode(PIN_U6, INPUT);
    digitalWrite(PIN_U6, HIGH);
    pinMode(PIN_U7, INPUT);
    digitalWrite(PIN_U7, HIGH);

    delay(200); // Calibration time for ultrasonics

    // Attaches interrupts to Ultrasonic pins
    // Interrupts when change in digital state
    attachInterrupt(PIN_U1, _ultrasonic_1_change, CHANGE);
    attachInterrupt(PIN_U2, _ultrasonic_2_change, CHANGE);
    attachInterrupt(PIN_U3, _ultrasonic_3_change, CHANGE);
    attachInterrupt(PIN_U4, _ultrasonic_4_change, CHANGE);
    attachInterrupt(PIN_U5, _ultrasonic_5_change, CHANGE);
    attachInterrupt(PIN_U6, _ultrasonic_6_change, CHANGE);
    attachInterrupt(PIN_U7, _ultrasonic_7_change, CHANGE);

 }

// ISR for Ultrasonics
void _ultrasonic_1_change(){
    if(digitalRead(PIN_U1) == HIGH){
        ultrasonic_1_time_rising = micros();
    }
    else{
        _filter_ultrasonic_1.add(micros() - ultrasonic_1_time_rising);
    }

}

void _ultrasonic_2_change(){
    if(digitalRead(PIN_U2) == HIGH){
        ultrasonic_2_time_rising = micros();
    }
    else{
        _filter_ultrasonic_2.add(micros() - ultrasonic_2_time_rising);
    }

}

void _ultrasonic_3_change(){
    if(digitalRead(PIN_U3) == HIGH){
        ultrasonic_3_time_rising = micros();
    }
    else{
        _filter_ultrasonic_3.add(micros() - ultrasonic_3_time_rising);
    }

}

void _ultrasonic_4_change(){
    if(digitalRead(PIN_U4) == HIGH){
        ultrasonic_4_time_rising = micros();
    }
    else{
        _filter_ultrasonic_4.add(micros() - ultrasonic_4_time_rising);
    }

}

void _ultrasonic_5_change(){
    if(digitalRead(PIN_U5) == HIGH){
        ultrasonic_5_time_rising = micros();
    }
    else{
        _filter_ultrasonic_5.add(micros() - ultrasonic_5_time_rising);
    }

}

void _ultrasonic_6_change(){
    if(digitalRead(PIN_U6) == HIGH){
        ultrasonic_6_time_rising = micros();
    }
    else{
        _filter_ultrasonic_6.add(micros() - ultrasonic_6_time_rising);
    }

}

void _ultrasonic_7_change(){
    if(digitalRead(PIN_U7) == HIGH){
        ultrasonic_7_time_rising = micros();
    }
    else{
        _filter_ultrasonic_7.add(micros() - ultrasonic_7_time_rising);
    }

}

// Sets trigger for ultrasonics
void read_ultrasonics(){

    // Trigger Ultrasonics
    digitalWrite(PIN_ULTRASONIC_TRIGGER,HIGH);
    delay(100);
    digitalWrite(PIN_ULTRASONIC_TRIGGER,LOW);

}

// Returns distance seen by ultrasonics in inches
// Distance is multiplied by a scaling factor to convert to inches, then converted to meters
float get_ultrasonic_1 () {
    return _filter_ultrasonic_1.getMedian() * ULTRASONIC_CORRECTION_FACTOR_7363 * INCHES_TO_METERS;
}

float get_ultrasonic_2 () {
    return _filter_ultrasonic_2.getMedian() * ULTRASONIC_CORRECTION_FACTOR_7363 * INCHES_TO_METERS;
}

float get_ultrasonic_3 () {
    return _filter_ultrasonic_3.getMedian() * ULTRASONIC_CORRECTION_FACTOR_7001 * INCHES_TO_METERS;
}

float get_ultrasonic_4 () {
    return _filter_ultrasonic_4.getMedian() * ULTRASONIC_CORRECTION_FACTOR_7363 * INCHES_TO_METERS;
}

float get_ultrasonic_5 () {
    return _filter_ultrasonic_5.getMedian() * ULTRASONIC_CORRECTION_FACTOR_7363 * INCHES_TO_METERS;
}

float get_ultrasonic_6 () {
    return _filter_ultrasonic_6.getMedian() * ULTRASONIC_CORRECTION_FACTOR_7363 * INCHES_TO_METERS;
}

float get_ultrasonic_7 () {
    return _filter_ultrasonic_7.getMedian() * ULTRASONIC_CORRECTION_FACTOR_7363 * INCHES_TO_METERS;
}

/*

// Gets the object x distance values from the ultrasonics
long get_ultrasonic_x1(){
    return 18 + ultrasonic_1_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * cos(u1_theta);
}

long get_ultrasonic_x2(){
    return 18 + ultrasonic_2_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * cos(u2_theta);
}

long get_ultrasonic_x3(){
    return 18 + ultrasonic_3_distance * ULTRASONIC_CORRECTION_FACTOR_7001;
}

long get_ultrasonic_x4(){
    return 18 + ultrasonic_4_distance * ULTRASONIC_CORRECTION_FACTOR_7363;
}

long get_ultrasonic_x5(){
    return 18 + ultrasonic_5_distance * ULTRASONIC_CORRECTION_FACTOR_7363;
}

long get_ultrasonic_x6(){
    return 18 + ultrasonic_6_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * cos(u6_theta);
}

long get_ultrasonic_x7(){
    return 18 + ultrasonic_7_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * cos(u7_theta);
}

// Gets the object y distance from the ultrasonics
long get_ultrasonic_y1(){
    return 20.5 + ultrasonic_1_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * sin(u1_theta);
}

long get_ultrasonic_y2(){
    return 16.6 + ultrasonic_2_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * sin(u2_theta);
}

long get_ultrasonic_y3(){
    return 10.5;
}

long get_ultrasonic_y4(){
    return 0;
}

long get_ultrasonic_y5(){
    return -10.5;
}

long get_ultrasonic_y6(){
    return -16.5 - ultrasonic_6_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * sin(u6_theta);
}

long get_ultrasonic_y7(){
    return -20.5 - ultrasonic_7_distance * ULTRASONIC_CORRECTION_FACTOR_7363 * sin(u7_theta);
}

*/
