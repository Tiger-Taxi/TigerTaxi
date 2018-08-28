/*
 * Main DUE 2
 * 
 * This is the main program for DUE 2, which receives data from the ultrasonics
 * and controls the light and sounder for the cart
 */

 #define USE_USBCON

#include "rosdue.h"
#include "ultrasonics.h"
#include "IMU.h"
#include "brake.h"
#include "brake_sensor.h"
#include "notification_control.h"
#include "wheel_encoder.h"


static ROSdue ros;

/* Ultrasonic Publishers */
static Publisher pub_usonic_1("sensor_ultrasonic_1");
static Publisher pub_usonic_2("sensor_ultrasonic_2");
static Publisher pub_usonic_3("sensor_ultrasonic_3");
static Publisher pub_usonic_4("sensor_ultrasonic_4");
static Publisher pub_usonic_5("sensor_ultrasonic_5");
static Publisher pub_usonic_6("sensor_ultrasonic_6");
static Publisher pub_usonic_7("sensor_ultrasonic_7");

/* IMU Publishers */
Publisher pub_imu_accel_x("sensor_imu_accel_x");
Publisher pub_imu_accel_y("sensor_imu_accel_y");
Publisher pub_imu_accel_z("sensor_imu_accel_z");
Publisher pub_imu_quat_w("sensor_imu_quat_w");
Publisher pub_imu_quat_x("sensor_imu_quat_x");
Publisher pub_imu_quat_y("sensor_imu_quat_y");
Publisher pub_imu_quat_z("sensor_imu_quat_z");
Publisher pub_imu_gyro_y("sensor_imu_gyro_y");
Publisher pub_imu_gyro_p("sensor_imu_gyro_p");
Publisher pub_imu_gyro_r("sensor_imu_gyro_r");

/* Wheel Encoders */
Publisher pub_wheel_encoder_left("sensor_wheel_left");
Publisher pub_wheel_encoder_right("sensor_wheel_right");

Publisher pub_brake_sensor("brake_sensor");
Publisher publisher_revert_manual("revert_manual");

/* Subscribers for light and sounder */
void sub_handler_controlmode (float f);
void sub_handler_stop_zone_clear (float f);
void sub_handler_brake_encoder (float f);
Subscriber subscriber_controlmode("apm_controlmode", &sub_handler_controlmode);
Subscriber subscriber_stop_zone_clear("stop_zone_clear", &sub_handler_stop_zone_clear);

/* Time for timing ultrasonics and IMU */
#define ULTRASONIC_INTERVAL 200
long time_last_ultrasonic = millis();

#define IMU_INTERVAL 20
long time_last_imu = millis();

#define ENCODER_INTERVAL 10
long time_last_encoder = millis();

//control mode switching
int brake_value = 0;
float current_mode = 0.0;
long time_last_switch = millis();

void setup() {
  
    ros.setup();
    ros.addSubscriber(subscriber_controlmode);
    ros.addSubscriber(subscriber_stop_zone_clear);
    
    setup_ultrasonics();
    setup_IMU();
    setup_Encoders();

    setup_brake();
    ros.addPublisher(pub_brake_sensor);
    ros.addPublisher(publisher_revert_manual);
    
    setup_notification_control();
    light(false);
    sounder(false);
}

void loop() {


    // Get subscriptions from ROS
    ros.spinOnce();
    read_brakes();
    brake_value = get_brakes();
    ros.publish(pub_brake_sensor, String(brake_value));
    
    if(current_mode==2.0 && brake_value>0){
     if(millis()-time_last_switch >= 1000){
      current_mode = 0.0;
      ros.publish(publisher_revert_manual, String(current_mode));
      time_last_switch = millis();
     }
    }
    /* Ultrasonics */
    //if (millis() - time_last_ultrasonic >= ULTRASONIC_INTERVAL) {
    //    publish_ultrasonics();
    //    time_last_ultrasonic = millis();
    //}

    /* IMU */
    if (millis() - time_last_imu >= IMU_INTERVAL) {
        publish_IMU();
        time_last_imu = millis();
    }

	if((millis() - time_last_encoder) >= ENCODER_INTERVAL){
		read_encoders();
		ros.publish(pub_wheel_encoder_left, String(GetLeftEncoder()));
		ros.publish(pub_wheel_encoder_right, String(GetRightEncoder()));
    time_last_encoder = millis();
	}

}
    


void publish_IMU () {
    read_IMU();
    ros.publish(pub_imu_accel_x, String(get_IMU_AccelX()));
    ros.publish(pub_imu_accel_y, String(get_IMU_AccelY()));
    ros.publish(pub_imu_accel_z, String(get_IMU_AccelZ()));
    ros.publish(pub_imu_quat_w, String(get_IMU_QuatW()));
    ros.publish(pub_imu_quat_x, String(get_IMU_QuatX()));
    ros.publish(pub_imu_quat_y, String(get_IMU_QuatY()));
    ros.publish(pub_imu_quat_z, String(get_IMU_QuatZ()));
    ros.publish(pub_imu_gyro_y, String(get_IMU_GyroY()));
    ros.publish(pub_imu_gyro_p, String(get_IMU_GyroP()));
    ros.publish(pub_imu_gyro_r, String(get_IMU_GyroR()));
}

/* Publishes the ultrasonic sensors */
void publish_ultrasonics() {
    read_ultrasonics();
    ros.publish(pub_usonic_1, String(get_ultrasonic_1()));
    ros.publish(pub_usonic_2, String(get_ultrasonic_2()));
    ros.publish(pub_usonic_3, String(get_ultrasonic_3()));
    ros.publish(pub_usonic_4, String(get_ultrasonic_4()));
    ros.publish(pub_usonic_5, String(get_ultrasonic_5()));
    ros.publish(pub_usonic_6, String(get_ultrasonic_6()));
    ros.publish(pub_usonic_7, String(get_ultrasonic_7()));
}

// Turns on the light if we are in autonomous
void sub_handler_controlmode (float f){
  light( ((int)f) == 2 );
  current_mode = f;
}

// Turns on the sounder if the stop zone is not clear
void sub_handler_stop_zone_clear (float f){
    sounder( ((int) f) == 0 );
}
