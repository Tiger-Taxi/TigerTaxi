#include "notification_control.h"

// set up notification control
void setup_notification_control(){
	pinMode(PIN_SOUNDER_CONTROL, OUTPUT);
	pinMode(PIN_LIGHT_CONTROL, OUTPUT);
}

//Turn on or off sounder
void sounder(bool on){
	if (on){
		digitalWrite(PIN_SOUNDER_CONTROL, HIGH);
	}
	else {
		digitalWrite(PIN_SOUNDER_CONTROL, LOW);
	}
}

//Turn on or off light
void light (bool on) {
	if (on) {
		digitalWrite(PIN_LIGHT_CONTROL, HIGH);
	}
	else {
		digitalWrite(PIN_LIGHT_CONTROL, LOW);
	}
}