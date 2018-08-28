#include "wheel_encoder.h"
#include <SPI.h> //SPI library
#include "Arduino.h"

//Slave Pins
const int slaveSelectEnc1 = 10;
const int slaveSelectEnc2 = 9;

//encoder counts
signed long encoder1count = 0;
signed long encoder2count = 0;

void clearEncoderCount(){
	//set encoder1 to 0
	digitalWrite(slaveSelectEnc1,LOW);
	SPI.transfer(0x20);//clear CNTR  lsb bits are 000
	//load data, MSB to LSB
	//SPI.transfer(0x00);
	//SPI.transfer(0x00);
	//SPI.transfer(0x00);
	//SPI.transfer(0x00);
	digitalWrite(slaveSelectEnc1, HIGH); //terminate SPI
	
	delayMicroseconds(100);
	
	//zero encoder 1
	//digitalWrite(slaveSelectEnc1,LOW);
	//SPI.transfer(0xE);  //0xE will clear MDR0... with dont cares of 110
	//digitalWrite(slaveSelectEnc1,HIGH);
	
	//set encoder 2 to 0
	digitalWrite(slaveSelectEnc2,LOW);
	SPI.transfer(0x20);//Clear CNTR
	//load data, MSB to LSB
	//SPI.transfer(0x00);
	//SPI.transfer(0x00);
	//SPI.transfer(0x00);
	//SPI.transfer(0x00);
	digitalWrite(slaveSelectEnc2, HIGH); //terminate SPI
	
	delayMicroseconds(100);
	
	//zero encoder 2
	//digitalWrite(slaveSelectEnc2,LOW);
	//SPI.transfer(0xE);  //0xE will clear MDR0... with dont cares of 110
	//digitalWrite(slaveSelectEnc2,HIGH);
}


void setup_Encoders() {

	//slave as outputs
	pinMode(slaveSelectEnc1, OUTPUT);
	pinMode(slaveSelectEnc2, OUTPUT);
	
	//raise select pins
	digitalWrite(slaveSelectEnc1, HIGH);
	digitalWrite(slaveSelectEnc2, HIGH);
	
	SPI.begin();
	
	//initialize first encoder
	digitalWrite(slaveSelectEnc1, LOW);
	SPI.transfer(0x88); //write to MDR0
	SPI.transfer(0x03); // x4 mode, free-running count, no index, async(don't care), 1 division factor
	digitalWrite(slaveSelectEnc1, HIGH); //terminate spi
	
	//initialize second encoder
	digitalWrite(slaveSelectEnc2, LOW);
	SPI.transfer(0x88); //write to MDR0
	SPI.transfer(0x03); // x4 mode, free-running count, no index, async(don't care), 1 division factor
	digitalWrite(slaveSelectEnc2, HIGH);

	clearEncoderCount();

	Serial.print("Encoders Setup");
}

long read_encoder(int encoder){
	unsigned int count_1, count_2, count_3, count_4;
	long count_value;
	
	//encoder1
	if (encoder == 1){
		digitalWrite(slaveSelectEnc1,LOW); //start SPI conversation
		SPI.transfer(0x60); //Request count
		count_1 = SPI.transfer(0x00);
		count_2 = SPI.transfer(0x00);
		count_3 = SPI.transfer(0x00);
		count_4 = SPI.transfer(0x00); //read lowest order byte
		digitalWrite (slaveSelectEnc1,HIGH);//terminate SPI
	}
	else if(encoder == 2){
		digitalWrite(slaveSelectEnc2,LOW); //start SPI conversation
		SPI.transfer(0x60); //Request count
		count_1 = SPI.transfer(0x00);
		count_2 = SPI.transfer(0x00);
		count_3 = SPI.transfer(0x00);
		count_4 = SPI.transfer(0x00); //read lowest order byte
		digitalWrite (slaveSelectEnc2,HIGH);//terminate SPI
	}
	//calculate count value
	count_value = (count_1<<8) + count_2;
	count_value = (count_value<<8) + count_3;
	count_value = (count_value<<8) + count_4;
	
	return count_value;
}

/*
void setup() {
	Serial.begin(9600);
	
	setupEncoders();	Serial.println("Encoders Initialized");
	clearEncoderCount();	Serial.println("Encoders Cleared");
}

void loop() {
	delay(500);
	
	encoder1count = readEncoder(1);
	encoder2count = readEncoder(2);

	//output encoder info here
	Serial.print("Enc1: "): Serial.print(encoder1count); Serial.print(" Enc2: "); Serial.println(encoder2count);
}
*/
void read_encoders(){
	encoder1count = read_encoder(1);
	encoder2count = read_encoder(2);
}

long GetLeftEncoder(){
	return encoder1count;
}
long GetRightEncoder(){
	return encoder2count;
}
