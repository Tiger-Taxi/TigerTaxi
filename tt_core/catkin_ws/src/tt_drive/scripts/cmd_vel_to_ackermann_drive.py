#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

LUCY_THROT_OFFSET = 23
EPSILON = 0.1

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def cmd_callback(data):
    global wheelbase
    global ackermann_cmd_topic
    global frame_id
    global pub_ack
    global pub_steer
    global pub_throttle
    
    v = data.linear.x
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
    
    msg_ack = AckermannDriveStamped()
    msg_ack.header.stamp = rospy.Time.now()
    msg_ack.header.frame_id = frame_id
    msg_ack.drive.steering_angle = steering
    msg_ack.drive.speed = v
    
    pub_ack.publish(msg_ack)

    msg_steer = Float32()
    if v < 0:
        msg_steer.data = -steering*180.0/3.14159262
    else:
        msg_steer.data = steering*180.0/3.14159262
    
    pub_steer.publish(msg_steer)

    msg_throttle = Float32()
    if v > (0 + EPSILON) and v < (LUCY_THROT_OFFSET / 37.3):
        v = LUCY_THROT_OFFSET
    elif v > (LUCY_THROT_OFFSET / 37.3):
        v = v * 37.3
    else:
        v = 0
    
    msg_throttle.data = v

    pub_throttle.publish(msg_throttle)


if __name__ == '__main__': 
    try:
        
        rospy.init_node('cmd_vel_to_ackermann_drive')
                
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
        ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')

        cmd_steering_topic =    rospy.get_param('~cmd_steering_topic', '/cmd_steering')
        cmd_throttle_topic = rospy.get_param('~cmd_throttle_topic', '/cmd_throttle')

        wheelbase = rospy.get_param('~wheelbase', 1.63)
        frame_id = rospy.get_param('~frame_id', 'odom')
        
        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
        pub_ack = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)

        pub_steer = rospy.Publisher(cmd_steering_topic, Float32, queue_size=1)
        pub_throttle = rospy.Publisher(cmd_throttle_topic, Float32, queue_size=1)
        
        rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
