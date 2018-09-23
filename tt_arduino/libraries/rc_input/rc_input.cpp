#include "rc_input.h"

void setup_rc() {
    pinMode(PIN_EVELATOR_IN, INPUT);
    pinMode(PIN_AILERON_IN, INPUT);
    pinMode(PIN_ESTOP_IN, INPUT);
    attachInterrupt(PIN_EVELATOR_IN, _measure_pulse_elevator, CHANGE);
    attachInterrupt(PIN_AILERON_IN, _measure_pulse_aileron, CHANGE);
    attachInterrupt(PIN_ESTOP_IN, _measure_pulse_estop, CHANGE);
}

// Reads and saves the present value of the steering
RC_commands get_rc_commands() {

    RC_commands current_commands;
    current_commands.throttle = _get_throttle_value(elevator_pulse_width);
    current_commands.brake = _get_brake_value(elevator_pulse_width);
    current_commands.steering = _get_steering_value(aileron_pulse_width);
    current_commands.estop = _get_estop_value(estop_pulse_width);

    return current_commands;
}

// Given a raw throttle value, gets the throttle percentage
double _get_throttle_value (int elevator_pulse_width) {
  if (elevator_pulse_width >= THROTTLE_MAX) return 100;
  else if (elevator_pulse_width <= THROTTLE_MIN) return 0;
  else {
      return ((double)(elevator_pulse_width - THROTTLE_MIN)/(THROTTLE_MAX - THROTTLE_MIN)) * 100;
  }
}

// Given a raw brake value, gets the brake percentage
double _get_brake_value (int elevator_pulse_width) {
   if (elevator_pulse_width >= BRAKE_MAX) return 0;
   else if (elevator_pulse_width <= BRAKE_MIN ) return 100;
   else {
      return 100 - ((double)(elevator_pulse_width - BRAKE_MIN)/(BRAKE_MAX - BRAKE_MIN)) * 100;
   }
}

// Given a raw steering value, gets the steering percentage
double _get_steering_value (int aileron_pulse_width) {
  if (aileron_pulse_width >= STEERING_MAX) return 0;
  else if(aileron_pulse_width <= STEERING_MIN) return 100;
  else {
    return 100 - ((double)(aileron_pulse_width - STEERING_MIN)/(STEERING_MAX - STEERING_MIN))*100;
  }
}

bool _get_estop_value (int estop_pulse_width) {
    // estop min and max are 1080 and 1926
    return estop_pulse_width < 1500;
}

void _measure_pulse_elevator () {
    if(digitalRead(PIN_EVELATOR_IN))
        elevator_trig = micros();
    else
        elevator_pulse_width = micros() - elevator_trig;
}

void _measure_pulse_aileron () {
    if(digitalRead(PIN_AILERON_IN))
        aileron_trig = micros();
    else
        aileron_pulse_width = micros() - aileron_trig;
}

void _measure_pulse_estop () {
    if(digitalRead(PIN_ESTOP_IN))
        estop_trig = micros();
    else
        estop_pulse_width = micros() - estop_trig;
}
