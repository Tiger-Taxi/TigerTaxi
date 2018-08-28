#ifndef NOTIF_CONT_H
#define NOTIF_CONT_H

#include "Arduino.h"

#define PIN_SOUNDER_CONTROL 32
#define PIN_LIGHT_CONTROL 34

void setup_notification_control();

//Turn on or off sounder
void sounder (bool on);

//Turn on or off light
void light (bool on);

#endif