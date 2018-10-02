#include "steering.h"

// Setup 333hz pwm on pin 8
arduino_due::pwm_lib::pwm<arduino_due::pwm_lib::pwm_pin::PWML5_PC22> pwm_pin8;

// Setup function for the steering
void setup_steering () {
    analogReadResolution(12);
    analogWriteResolution(12);
    // Set Talon SR PWM pin to 333 Hz with 50% duty cycle
    pwm_pin8.start(TALON_PERIOD * TALON_PERIOD_SCALING, TALON_CENTER * TALON_PERIOD_SCALING);
    setup_PID();
}

// Reads and saves the present value of the steering
int get_position_steering () {
    //steering_averager.addValue(analogRead(PIN_STEERING_POT));
    //current_steering_pot_value = steering_averager.getAverage();
    current_steering_pot_value = analogRead(PIN_STEERING_POT);
    return current_steering_pot_value;
}

// Sets up the PID system
void setup_PID () {
    // steering_PID.SetOutputLimits(-STEERING_CORRECTION_MAX, STEERING_CORRECTION_MAX);
    steering_PID.SetOutputLimits(-TALON_CORRECTION_MAX, TALON_CORRECTION_MAX);
    steering_PID.SetSampleTime(STEERING_PID_SAMPLE_TIME);
    steering_PID.SetMode(AUTOMATIC);
}

// Sets the steering rack to a potentiometer value
void set_steering_pot_value(double pot_val) {

    // Update the steering pot value
    get_position_steering();

    // Set the PID desired to the pot_val
    if (pot_val > STEERING_POT_MAX) {
        desired_steering_pot_value = STEERING_POT_MAX;
    } else if ( pot_val < STEERING_POT_MIN) {
        desired_steering_pot_value = STEERING_POT_MIN;
    } else {
        desired_steering_pot_value = pot_val;
    }

    // Compute the correction with PID
    steering_PID.Compute();

    // Take the output of the PID (correction) and write to the DAC
    // analogWrite(PIN_STEERING_OUT, (STEER_CENTER - steering_pid_output));

    // Take the output of the PID (correction) and send a PWM to the Talon SR
    pwm_pin8.set_duty((TALON_CENTER + steering_pid_output) * TALON_PERIOD_SCALING);
}

// Set the desired steering angle
void set_steering_angle(float steering_angle) {

    // Convert from angle to pot value, and set steering to that pot value
    set_steering_pot_value(STEERING_POT_CENTER - ((float)steering_angle/STEERING_ANGLE_FACTOR));

}

// Set the desired position for the steering,
// Where from driver's seat 0 is full left, 100 is full right, and 50 is in the middle
void set_steering_percentage (double steering_percentage) {

    // Compute the desired pot value for that given
    set_steering_pot_value((STEERING_M * steering_percentage) + STEERING_POT_MIN);

}

void steer_left() {
    analogWrite(PIN_STEERING_OUT, STEER_CENTER + STEERING_CORRECTION_MAX);
}

void steer_right() {
    analogWrite(PIN_STEERING_OUT, STEER_CENTER - STEERING_CORRECTION_MAX);
}

void stop_steering() {
    // analogWrite(PIN_STEERING_OUT, STEER_CENTER);
    pwm_pin8.set_duty(0);
}
