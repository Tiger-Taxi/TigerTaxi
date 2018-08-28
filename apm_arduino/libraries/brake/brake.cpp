#include "brake.h"
#include <stdlib.h>

// Setup function for the brake
void setup_brake() {
    analogReadResolution(12);
    brake_actuator.attach(PIN_BRAKE_OUT, PULSE_MIN, PULSE_MAX);
}

// Reads, saves, and returns the present value of the brake
int get_position_brake() {
    brake_averager.addValue(analogRead(PIN_BRAKE_POT_IN));
    current_brake_position = brake_averager.getAverage();
    return current_brake_position;
}

// Extends the brake
void brake_extend() {
    brake_actuator.write(BRAKE_EXTEND);
}

// Retracts the brake
void brake_retract () {
    brake_actuator.write(BRAKE_RETRACT);
}

// Keeps the brake where it is
void brake_stay () {
    brake_actuator.write(BRAKE_STAY);
}

/*
 * Sets the brake actuator position over time.
 * Designed to be run as often as possible
 */
void set_brake(double brake_amount) {

    // Update our current position
    get_position_brake();

    // See if it is time to evaluate another command
    if ( (millis() - time_since_last_command) >= 20) {

        // Calculate desired_brake_position based on brake_amount percentage
        // We do this to convert a brake effort from 0 - 100 into pot values
        brake_amount = abs(brake_amount - 100);
        int desired_brake_position = (int) ((brake_amount * BRAKE_POS_MAX) / 100);

        // Calculate how far off desired we are
        int delta = desired_brake_position - current_brake_position;
        int delta_absolute = abs(delta);

        // Check what region we are in,
        // If we have changed regions update the delay amount and delay count
        if (delta_absolute <= DAMPING_REGION_0_DELTA && delay_amount != DAMPING_REGION_0_DELAY) {
            delay_amount = DAMPING_REGION_0_DELAY;
            delay_cycles = delay_amount;
        } else if (delta_absolute <= DAMPING_REGION_1_DELTA && delay_amount != DAMPING_REGION_1_DELAY) {
            delay_amount = DAMPING_REGION_1_DELAY;
            delay_cycles = delay_amount;
        } else if (delta_absolute > DAMPING_REGION_1_DELTA && delay_amount != 0) {
            delay_amount = 0;
            delay_cycles = 0;
        }

        // Send our command to the actuator
        if (delay_cycles != 0) {

            // Delay cycle
            brake_stay();
            delay_cycles--;

        } else {

            if (delta >= 1) {
                brake_extend();
            }  else if (delta <= -1) {
                brake_retract();
            } else {
                brake_stay();
            }

            // Reset delay cycles
            delay_cycles = delay_amount;
        }

        time_since_last_command = millis();
    }

}
