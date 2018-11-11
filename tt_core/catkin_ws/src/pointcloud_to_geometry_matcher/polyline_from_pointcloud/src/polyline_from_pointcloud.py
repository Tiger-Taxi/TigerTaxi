#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
from sensor_msgs import point_cloud2

from rdp import rdp

latest_pointcloud = None


def pointcloud_callback(data):
    latest_pointcloud = point_cloud2.read_points(data)


def run_node():
    rospy.init_node('polyline_from_pointcloud')
    rate = rospy.Rate(10)  # Hz

    rospy.Subscriber("/velodyne_points", PointCloud2, pointcloud_callback)
    poly_pub = rospy.Publisher("/velodyne_polyline", PointCloud2)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        run_node()
    except rospy.ROSInterruptException:
        pass
