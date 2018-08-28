/*--------------------------------------------------------------
  Program:      brake_sensor

  Description:  Reads value of sensor attached to arduino board
  
  Hardware:     Arduino Uno with digital sensor

  Date:         21 January 2016
 
  Author:       Ryan Trumpore
--------------------------------------------------------------*/
const int brakeSensor = 50; //pin with sensor

int brakeState = -1; //state of brake

void setup() {

	Serial.begin(9600);
	analogWrite(8,255);
	pinMode(brakeSensor, INPUT);
}

void read_brakes(){
	brakeState = digitalRead(brakeSensor);
}

int get_brakes(){
	return brakeState;
}

void loop(){
  read_brakes();
  Serial.println(brakeState);
}


