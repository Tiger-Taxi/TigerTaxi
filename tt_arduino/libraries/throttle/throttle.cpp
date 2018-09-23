#include "throttle.h"

// Setup function for the throttle
void setup_throttle() {
    analogReadResolution(12);
    analogWriteResolution(12);
}

// Returns the present percentage of the throttle pedal
double get_position_throttle () {
    throttle_sensor_value = (analogRead(PIN_THROTTLE_IN)/4095.0) * 100;
    return throttle_sensor_value;
}

// Set the value of the throttle percentage
void set_throttle (double throttle) {
  // Convert the throttle percentage to a DAC input between 0 and 4095 (because it's a 12-bit DAC)
  analogWrite(PIN_THROTTLE_OUT, (throttle*40.95)/2);
}
