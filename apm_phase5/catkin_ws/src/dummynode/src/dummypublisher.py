#!/usr/bin/env python
#this is a test module
import rospy
from sensor_msgs.msg import NavSatFix

def talker():
    pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
    rospy.init_node('dummynode', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    longtude = -77.6822
    while not rospy.is_shutdown():
	msgGPS = NavSatFix()
        msgGPS.latitude = 43.083
        msgGPS.longitude = longtude
        msgGPS.altitude = 0
        pub.publish(msgGPS)
        rate.sleep()
	longtude += 0.0001
	if longtude > -77.6737:
		longtude = -77.6822

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
