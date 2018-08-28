/*
 * Main DUE 1
 *
 * This is the main program for DUE 1, which controls the throttle, steering, and brakes.
 */

#define USE_USBCON
 
#include "brake.h"
#include "throttle.h"
#include "steering.h"
#include "brake_sensor.h"
#include "rc_input.h"
#include "rosdue.h"
#include "cart_odom.h"
#include "error_indication.h"
/* Subscriber handlers */
void sub_handler_controlmode (float f);
void sub_handler_brake_encoder (float f);
void sub_handler_steering_encoder (float f);
void sub_handler_stop_zone_clear (float f);
void sub_handler_cmd_steering(float f);
void sub_handler_cmd_throttle(float f);
void sub_handler_cmd_brake(float f);
/* ROS connection */
ROSdue ros;
Subscriber subscriber_controlmode("apm_controlmode", &sub_handler_controlmode);
Subscriber subscriber_stop_zone_clear("stop_zone_clear", &sub_handler_stop_zone_clear);
Subscriber subscriber_cmd_steering("cmd_steering", &sub_handler_cmd_steering);
Subscriber subscriber_cmd_throttle("cmd_throttle", &sub_handler_cmd_throttle);
Subscriber subscriber_cmd_brake("cmd_brake", &sub_handler_cmd_brake);
Publisher publisher_remote_cmd_steering("remote_control_cmd_steering");
Publisher publisher_remote_cmd_throttle("remote_control_cmd_throttle");
Publisher publisher_remote_cmd_brake("remote_control_cmd_brake");
Publisher publisher_revert_manual("revert_manual");
Publisher publisher_control_loop_time("control_loop_time");
/* Control Mode */
typedef enum ControlMode { STOCK, REMOTE, AUTONOMOUS, ISSUE };
ControlMode current_mode = STOCK;
int brake_value = 0;
int steering_value = 0;
double throttle_value = 0;
double throttleControl[20];
int throttleIndex = 0;
float loopTimer = millis();
float manualTimer = millis();

/* Autonomous Control Values */
volatile float cmd_throttle = 0;
volatile float cmd_steering = 0;
volatile float cmd_brake = 0;
/* Stop-zone state */
static bool stop_zone_clear = true;
/* The current RC command */
RC_commands current_command;
void setup() {
    Serial.print("Setup started");

    // Subsystems
    setup_throttle();
    setup_steering();
    setup_brake();
    // Modes
    setup_remote();
    setup_autonomous();
    //ROSdue
    Serial.print("Setup niddled");

    ros.setup();
    ros.addSubscriber(subscriber_controlmode);
    ros.addSubscriber(subscriber_stop_zone_clear);
    ros.addSubscriber(subscriber_cmd_throttle);
    ros.addSubscriber(subscriber_cmd_steering);
    ros.addSubscriber(subscriber_cmd_brake);
    ros.addPublisher(publisher_remote_cmd_steering);
    ros.addPublisher(publisher_remote_cmd_throttle);
    ros.addPublisher(publisher_remote_cmd_brake);
    ros.addPublisher(publisher_revert_manual);
    ros.addPublisher(publisher_control_loop_time);
    // Cart Odometry
    setup_cart_odom(ros);
    // Error Indication
    setup_error_indication();
    Serial.print("Setup Completed");
}
// Run an iteration of controls in manual mode
void run_manual () {
    // Pass through the throttle to the output
    set_throttle(get_position_throttle());
    // Turn off the brake
    set_brake(0);
    // TODO pass-through the torque sensors
    // In the meantime, do nothing with the steering
    stop_steering();
}
// Sets up the remote control mode
void setup_remote () {
    setup_rc();
}
// Run an iteration of controls in remote mode
void run_remote () {
    if (stop_zone_clear){
      Serial.print("Stop_zone_clear:True\r\n");
      Serial.print(current_command.steering);
      Serial.print("\r\n");
      Serial.print(current_command.brake);
      Serial.print("\r\n");

      set_throttle(current_command.throttle);
      set_brake(current_command.brake);
      set_steering_percentage(current_command.steering);
      
    } else {
      Serial.print("Stop_zone_clear:False\r\n");
      set_throttle(0);
      set_brake(100);
      stop_steering();
    
    }
 
}
// Sets up the autonomous mode
void setup_autonomous () {
    // TODO
}
// Run an iteration of controls in autonomous mode
void run_autonomous () {
    // Publish remote commands for desired vector (for now!)
    current_command = get_rc_commands();
    ros.publish(publisher_remote_cmd_steering, String(get_steering_percent_to_angle(current_command.steering)));
    ros.publish(publisher_remote_cmd_throttle, String(current_command.throttle));
    ros.publish(publisher_remote_cmd_brake, String(current_command.brake));
    // OK to go
    if (stop_zone_clear) {
      
      set_throttle(cmd_throttle);
      set_steering_angle(cmd_steering);
      set_brake(cmd_brake);
    // Stop zone is not clear, come to a stop!
    } else {
      ros.publish(publisher_control_loop_time,"stop_zone_clear");
      set_throttle(0);
      stop_steering();
      set_brake(100);
    
    }
    
}
void loop() {
    Serial.print("loop started\r\n");  
    Serial.print("current_command.estop");
    Serial.print(current_command.estop);
    Serial.print("\r\ncurrent_mode");
    Serial.print(current_mode);
    Serial.print("\r\n");
    loopTimer = millis();
    // Get subscriptions from ROS
    ros.spinOnce();
    Serial.print("post spinOnce\r\n");
    // Updates the cart odom
    cart_odom_spin();
    // Check for remote e-stop
    current_command = get_rc_commands();
    // Check brake sensor for brake
    read_brakes();
    brake_value = get_brakes();
  
    // Check pedal for throttle
    throttle_value = get_position_throttle();
   
    throttleControl[throttleIndex%20] = throttle_value;
    throttleIndex++;
    throttle_value = 0;
    for(int i=0; i<21;i++){
      throttle_value += throttleControl[i];
    }
    throttle_value = throttle_value/20;  
  
    if(throttle_value>20 && current_mode != STOCK) {
      Serial.print("Resetting to STOCK");
      current_mode = STOCK;
      if(loopTimer-manualTimer>1000 ){
        //ros.publish(publisher_revert_manual, String(current_mode));
        manualTimer=loopTimer;
      }
    }

    
    
    ros.publish(publisher_control_loop_time, String(current_mode)); 
    
    if (current_command.estop || current_mode == ISSUE) {
        // Stop cart
        set_throttle(0);
        set_brake(100);
        stop_steering();
        // Turn on error LED and sounder
        error_1(true);
        error_sounder(true);
        
    } else {
        // Normal operation
        error_1(false);
        error_sounder(false);
        
        switch (current_mode) {
      
            case STOCK:
                Serial.print("STOCK\r\n");
                run_manual();
            break;
      
            case REMOTE:
                Serial.print("REMOTE\r\n");
                run_remote();
            break;
      
            case AUTONOMOUS:
                Serial.print("AUTONOMOUS\r\n");
                run_autonomous();
            break;
      
        } 
    
    }
    loopTimer = millis()-loopTimer;
}
/* Handler for control mode change */
void sub_handler_controlmode (float f){
    switch ((int) f) {
        case 0:
            current_mode = STOCK;
            break;
        case 1:
            current_mode = REMOTE;
            break;
        case 2:
            current_mode = AUTONOMOUS;
            break;
        default:
            current_mode = ISSUE;
            break;
    }
}
void sub_handler_stop_zone_clear (float f){
    if ((int)f == 1) {
      stop_zone_clear = true;
    } else {
      stop_zone_clear = false;
    }
}
void sub_handler_cmd_steering(float f){
  cmd_steering = f;
}
void sub_handler_cmd_throttle(float f){
  cmd_throttle = f;
}
void sub_handler_cmd_brake(float f){
  cmd_brake = f;
}

