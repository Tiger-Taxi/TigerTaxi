
#include "IMU.h"

// Set up IMU
void setup_IMU(){

	Wire.begin();
	
	accelgyro.initialize();
	ret = mympu_open(400);
}


void read_IMU(){
	
	ret = mympu_update();
	getAccel_Data();
	getCompassDate_calibrated(); 
	getHeading();

}


void getAccel_Data(){

	accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	Axyz[0] = (double)ax*9.80665 / 16384;    //convert G's to m/s^2
	Axyz[1] = (double)ay*9.80665 / 16384;
	Axyz[2] = (double)az*9.80665 / 16384;
}



// Get IMU values
float get_IMU_AccelX(){
	return Axyz[0];
}
float get_IMU_AccelY(){
	return Axyz[1];
}
float get_IMU_AccelZ(){
	return Axyz[2];
}
float get_IMU_QuatW(){
	return mympu.quat[0];
}
float get_IMU_QuatX(){
	return mympu.quat[1];
}
float get_IMU_QuatY(){
	return mympu.quat[2];
}
float get_IMU_QuatZ(){
	return mympu.quat[3];
}
float get_IMU_GyroY(){
	return mympu.gyro[0];
}
float get_IMU_GyroP(){
	return mympu.gyro[1];
}
float get_IMU_GyroR(){
	return mympu.gyro[2];
}

float get_IMU_Heading(){
	return heading;
}

//Get heading
void getHeading(){
	heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
	if (heading < 0) heading += 360;
}



//Compass Calibration
void Mxyz_init_calibrated()
{
	get_calibration_Data();

}

void get_calibration_Data(){
	for (int i = 0; i < sample_num_mdate; i++)
	{
		get_one_sample_date_mxyz();
		/*
		Serial.print(mx_sample[2]);
		Serial.print(" ");
		Serial.print(my_sample[2]);                            //you can see the sample data here .
		Serial.print(" ");
		Serial.println(mz_sample[2]);
		*/

		if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
		if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
		if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

		if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
		if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
		if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

	}

	mx_max = mx_sample[1];
	my_max = my_sample[1];
	mz_max = mz_sample[1];

	mx_min = mx_sample[0];
	my_min = my_sample[0];
	mz_min = mz_sample[0];



	mx_centre = (mx_max + mx_min) / 2;
	my_centre = (my_max + my_min) / 2;
	mz_centre = (mz_max + mz_min) / 2;


}


void get_one_sample_date_mxyz()
{
	getCompass_Data();
	mx_sample[2] = Mxyz[0];
	my_sample[2] = Mxyz[1];
	mz_sample[2] = Mxyz[2];
}


void getCompass_Data(void)
{
	I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
	delay(10);
	I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

	mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0];
	my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2];
	mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4];

	Mxyz[0] = (double)mx * 1200 / 4096;
	Mxyz[1] = (double)my * 1200 / 4096;
	Mxyz[2] = (double)mz * 1200 / 4096;
}


void getCompassDate_calibrated()
{
	getCompass_Data();
	Mxyz[0] = Mxyz[0] - mx_centre;
	Mxyz[1] = Mxyz[1] - my_centre;
	Mxyz[2] = Mxyz[2] - mz_centre;
}
