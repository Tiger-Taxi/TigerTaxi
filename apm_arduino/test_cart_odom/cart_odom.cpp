#include "cart_odom.h"
#include "wheel_encoder.h"
#include "brake_sensor.h"

// The last time we sent out an odom message
long odom_time_last_message;
long odom_elapsed_time;

// Speed: The number of vtach_ticks and time since we last evaluated speed
volatile long vtach_ticks = 0;

// The speed of the cart in m/s
volatile float cart_speed = 0.0;

// The angle of the cart in degrees
// zero = center
// Positive = driver's side
// Negative = passenger side
float cart_steering_angle = 0.0;

// ROSdue
ROSdue odom_ros;
Publisher publisher_speed("cart_odom_speed");
Publisher publisher_steering("cart_odom_steering");

// TODO temp remove this 
Publisher pub_wheel_encoder_left("sensor_wheel_left");
Publisher pub_wheel_encoder_right("sensor_wheel_right");

//TODO temp
Publisher publisher_brake_sensor("brake_sensor");

void setup_cart_odom(ROSdue r) {
    odom_ros = r;
    attachInterrupt(digitalPinToInterrupt(PIN_VTACH), _vtach_tick, CHANGE);
    odom_time_last_message = micros();
    analogReadResolution(12);
}

void publish_encoders() {
  read_encoders();
  odom_ros.publish(pub_wheel_encoder_left, String(GetLeftEncoder()));
  odom_ros.publish(pub_wheel_encoder_right, String(GetRightEncoder()));
}

void publish_brake_sensor(){
	read_brakes();
	odom_ros.publish(publisher_brake_sensor, String(get_brakes()));
	
}

void cart_odom_spin () {

    odom_elapsed_time = micros() - odom_time_last_message;

    // If enough time has elapsed between last time we sent
    if (odom_elapsed_time >= ODOM_PERIOD ){

        // Calculate the cart speed
        cart_speed = (vtach_ticks * VTACH_TICK_DISTANCE) / (odom_elapsed_time * 0.000001);

        // Make sure we aren't going to send noise
        if (cart_speed < VALID_SPEED_THRESHOLD) {

            // Output to ROSdue
            odom_ros.publish(publisher_speed, String(cart_speed, 8));

        }

        // Calculate steering angle
        cart_steering_angle = (STEERING_POT_CENTER - get_position_steering()) * STEERING_ANGLE_FACTOR;
        odom_ros.publish(publisher_steering, String(cart_steering_angle, 8));

        //publish the encoder topics
        publish_encoders();

		//publish the brake topics
		publish_brake_sensor();
		
        // Reset time
        odom_time_last_message = micros();

        // Reset ticks
        vtach_ticks = 0;

    }
}

float get_cart_speed() {
    return cart_speed;
}

// Convert from a desired steering percent to a desired steering angle
float get_steering_percent_to_angle(float percent) {
    return -1 * STEERING_ANGLE_FACTOR * ((STEERING_M * percent) + STEERING_POT_MIN - STEERING_POT_CENTER);
}

void _vtach_tick() {
    vtach_ticks++;
}
