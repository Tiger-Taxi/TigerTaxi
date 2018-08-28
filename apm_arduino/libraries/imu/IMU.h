#ifndef IMU_H
#define IMU_H

#include "mpu.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU9250.h"



static MPU9250 accelgyro;
static I2Cdev I2C_M;

static int16_t ax, ay, az;
static int16_t gx, gy, gz;
static int16_t mx, my, mz;
static float Axyz[3];
static float Mxyz[3];
static int ret;
static float heading;

//Compass Calibration
static uint8_t buffer_m[6];
static int sample_num_mdate = 5000;
static float mx_sample[3];
static float my_sample[3];
static float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

static int mx_max = 0;
static int my_max = 0;
static int mz_max = 0;

static int mx_min = 0;
static int my_min = 0;
static int mz_min = 0;

// Set Up IMU
void setup_IMU();
void Mxyz_init_calibrated(); //Run to calibrate compass, take ~2mins

// Read IMU values
void read_IMU();

void getAccel_Data();

void getCompass_Data();

// Get IMU values
float get_IMU_AccelX();
float get_IMU_AccelY();
float get_IMU_AccelZ();
float get_IMU_QuatW();
float get_IMU_QuatX();
float get_IMU_QuatY();
float get_IMU_QuatZ();
float get_IMU_GyroY();
float get_IMU_GyroP();
float get_IMU_GyroR();
float get_IMU_Heading();


//Compass Calibration
void get_calibration_Data();
void get_one_sample_date_mxyz();
void getCompassDate_calibrated();
void getHeading();


#endif
