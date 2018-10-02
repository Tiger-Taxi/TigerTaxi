/*--------------------------------------------------------------
  Program:      brake_sensor

  Description:  Reads value of sensor attached to arduino board
  
  Hardware:     Arduino Uno with digital sensor

  Date:         21 January 2016
 
  Author:       Ryan Trumpore
--------------------------------------------------------------*/
#include "brake_sensor.h"

int brakeSensor = 8; //pin with sensor

int brakeState = -1; //state of brake

void setup_Brakes() {
	
	pinMode(brakeSensor, INPUT);
}

void read_brakes(){
	brakeState = digitalRead(brakeSensor);
}

int get_brakes(){
	return brakeState;
}

