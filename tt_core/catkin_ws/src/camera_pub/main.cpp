/*
 *
 *  Created on: November 10, 2018
 *      Author: Kenneth Alexopoulos ksa6262@rit.edu
 *      
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <string>
#include <cstdlib>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_pub");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::VideoCapture cap("/dev/video0");
  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));

  

  ros::Rate loop_rate(30);
  while (nh.ok()) {
    cv::Mat new_frame;
    cap >> new_frame;
    cv::resize(new_frame, new_frame, cv::Size(600, 360),0,0,cv::INTER_NEAREST);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", new_frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}