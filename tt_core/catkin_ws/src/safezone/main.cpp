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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pthread.h>
#include <string>
#include <cstdlib>
#include <sensor_msgs/Image.h>
// Safezone includes
#include "include/ipm.hpp"


typedef pcl::PointCloud <pcl::PointXYZRGB> PointCloudT;

typedef enum pc_params{
  pcwidth = 80,
  pcheight = 45,
} pc_params;

void doItAll(const cv::Mat& image);

ros::Publisher safe_pub;
ros::Publisher unsafe_pub;
ros::Publisher canny_pub;
cv_bridge::CvImage out_msg;

void predicitonCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(msg);
        const cv::Mat pred = cv_image->image.clone();
        doItAll(pred);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Unable to convert %s image to cv", msg->encoding.c_str());
    }
}

//Convert Enet prediction to safe and unsafe point clouds
void doItAll(const cv::Mat& image) {
    ipm mapper;
    cv::Mat upsized_prediction;
    cv::resize(image, upsized_prediction, cv::Size(1920, 1080),0,0,cv::INTER_NEAREST);
    
    // Apply IPM
    upsized_prediction = mapper.removeDistortion(upsized_prediction);
    upsized_prediction = mapper.mapPerspective(upsized_prediction);
    // std::cout << upsized_prediction.type();
    
    // Edge Detection
    cv::Mat cloud_frame;
    cv::resize(upsized_prediction, cloud_frame, cv::Size(pcwidth, pcheight),0,0,cv::INTER_NEAREST);
    // Pad Bottom

    cv::Rect roi(0, 0, cloud_frame.cols, cloud_frame.rows);
    cv::copyMakeBorder(cloud_frame, cloud_frame, 0, 3, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));//this adds padding to the bottems
    // Split into BRG channels
    std::vector <cv::Mat> cloud_frame_spl;
    cv::split(cloud_frame, cloud_frame_spl);
    cv::Mat unsafe_mask;
    unsafe_mask = cloud_frame_spl[0].clone();//was 2
    // unsafe_mask = cloud_frame.clone();
    for(int i = 0; i < unsafe_mask.rows; i++){
        for (int j = 0; j < unsafe_mask.cols; ++j) {
            if (unsafe_mask.at<uchar>(i, j) != 1) {
                unsafe_mask.at<uchar>(i, j) = 255;
            }
        }
    }

    // cv::imwrite("/home/rosmaster/TigerTaxi/test/unsafemask.png", unsafe_mask);
    // cv::imwrite("/home/rosmaster/TigerTaxi/test/test.png", image);
    out_msg.encoding = "mono8"; // Or whatever
    out_msg.image    = unsafe_mask.clone(); // Your cv::Mat
    canny_pub.publish(out_msg.toImageMsg()); 
    // Run edge detection
    // cv::Canny(unsafe_mask.clone(), unsafe_mask, 1, 3, 3);
    // cv::imwrite("/home/rosmaster/TigerTaxi/test/b4roi.png", unsafe_mask);
    unsafe_mask = unsafe_mask(roi);

    

    
    // cv::imwrite("/home/rosmaster/TigerTaxi/test/afterroi.png", unsafe_mask);
 
    
    // cv::imwrite("/home/rosmaster/TigerTaxi/test/cannyout.png", unsafe_mask);
    // for(int i = 0; i < unsafe_mask.rows; i++)
    // {
    //     for (int j = 0; j < unsafe_mask.cols; ++j) {
    //         if (unsafe_mask.at<uchar>(i, j) != 0) {
    //                 std::cout << "nonzero values: " << static_cast<int>(unsafe_mask.at<uchar>(i, j)) << '\n';
    //         }
    //     }
    // }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr unsafe_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);

    safe_cloud->header.frame_id = "/camera";
    safe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    unsafe_cloud->header.frame_id = "/camera";
    unsafe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;
    //how for the camera sees from the cart
    float measured_height = 10.15;
    float measured_width = 19;

    // Construct pointcloud from perspective mapped image
    for (int r = 0; r < pcheight; ++r) {
        for (int c = 0; c < pcwidth; ++c) {
            if (unsafe_mask.at<uchar>(r, c) == 1) {
                // std::cout << "got an unsafe point!\n";
                pcl::PointXYZRGB p;
                //The coordinate of the point is taken from the depth map
                //Y and Z  taken negative to immediately visualize the cloud in the right way
                p.x = ((r - pcheight / 2.0) / (float) pcheight) * measured_height;
                p.y = ((c - pcwidth / 2.0) / (float) pcwidth) * measured_width;
                p.z = 0;

                // Color point for unsafe zones (BGR)
                p.r = 255;
                unsafe_cloud->points.push_back(p);
            } else {
                pcl::PointXYZRGB p;
                //The coordinate of the point is taken from the depth map
                //Y and Z  taken negative to immediately visualize the cloud in the right way
                p.x = ((r - pcheight / 2.0) / (float) pcheight) * measured_height;
                p.y = ((c - pcwidth / 2.0) / (float) pcwidth) * measured_width;
                p.z = 0;

                // Color point for unsafe zones (BGR)
                p.b = 255;
                safe_cloud->points.push_back(p);
            }
        }
    }
    
    safe_cloud->width = (int) safe_cloud->points.size();
    safe_cloud->height = 1;

    unsafe_cloud->width = (int) unsafe_cloud->points.size();
    unsafe_cloud->height = 1;

    safe_pub.publish(safe_cloud);
    unsafe_pub.publish(unsafe_cloud);
    // std::cout << "point clouds published!\n";
}

int main(int argc, char **argv) {

    // ROS node initialization
    ros::init(argc, argv, "safezone");
    ros::NodeHandle n;
    safe_pub = n.advertise<PointCloudT>("safezone_pc", 1);
    unsafe_pub = n.advertise<PointCloudT>("unsafezone_pc", 1);
    canny_pub = n.advertise<sensor_msgs::Image>("cannyimg", 1);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber it_sub = it.subscribe("/enet/image", 1, &predicitonCallback);
    
    ros::spin();
}