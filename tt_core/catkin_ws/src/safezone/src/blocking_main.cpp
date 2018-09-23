/*
 *
 *  Created on: Feb 5, 2018
 *      Author: Robert Relyea rer3378@rit.edu
 *
 */

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pthread.h>
#include <string>

// Safezone includes
#include "classifier.hpp"
#include "ipm.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

// Pointcloud publish params
typedef enum pc_params
{
  pcwidth = 80,
  pcheight = 45,
}pc_params;

// Safe area mode.
// 0 = road safe
// 1 = sidewalk safe
// 2 = road and sidewalk safe
float mode = 0.0;

// Mat variables for storing images
// cv::Mat img; // Create mat to hold new frames
// cv::Mat enet_frame, output_frame;

// Mats for storing image operations
// frame:       New frame captured by camera
// enet_frame:  Resulting segmented camera frame
// ipm_frame:   Resulting perspective mapped segmented frame
cv::Mat frame, enet_frame, ipm_frame;

// Mats for generating preview image
cv::Mat top_preview, bottom_preview, preview;

// Mutexes for locking intermediate image operation matricies
pthread_mutex_t frame_locker;
pthread_mutex_t enet_locker;
pthread_mutex_t ipm_locker;
pthread_mutex_t preview_locker;

pthread_barrier_t camera_done;
pthread_barrier_t enet_done;
pthread_barrier_t ipm_done;
pthread_barrier_t pc_done;

/* Function for camera capture thread
*/
void *cameraThread(void *arg)
{
  // Video capture initialization
  // cv::VideoCapture cap("/home/apm/Videos/Webcam/2018-04-02-171452.webm");
  cv::VideoCapture cap("/dev/video0");
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
  cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
  // cv::VideoCapture cap("/home/apm/Videos/1.mp4");

  while(true)
  {
    // pthread_mutex_lock(&frame_locker);
    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    cap >> frame;  // Grab new frame

    gettimeofday(&time, NULL);  //END-TIME
    totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);
    std::cout << "Camera time = " << totalTime << " ms" << std::endl;

    cv::resize(frame, frame, cv::Size(frameWidth, frameHeight));
    // pthread_mutex_unlock(&frame_locker);
    // cv::imshow("frame", frame);
    // cv::waitKey(1);


    
    pthread_barrier_wait(&camera_done);
    pthread_barrier_wait(&enet_done);
  }
}

void *enetThread(void *arg)
{
  // ENet initialization
  string model_file   = "/home/apm/tt_core/ENet/final_model_weights/bn_conv_merged_model.prototxt";
  string trained_file = "/home/apm/tt_core/ENet/final_model_weights/bn_conv_merged_weights.caffemodel";
  string LUT_file = "/home/apm/tt_core/safezone.png";
  Classifier classifier(model_file, trained_file);

  while(true)
  {
    pthread_barrier_wait(&camera_done);

    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    // Grab latest frame
    // pthread_mutex_lock(&frame_locker);

    // cv::Mat latest_frame = frame.clone();
    // // pthread_mutex_unlock(&frame_locker);

    // // Ensure frame is populated before processing
    // if(latest_frame.empty())
    //     continue;

    // Pass latest frame through ENet
    // pthread_mutex_lock(&enet_locker);
    enet_frame = classifier.Predict(frame, LUT_file);
    // pthread_mutex_unlock(&enet_locker);
    cv::hconcat(frame, enet_frame, top_preview);
    cv::imshow("Preview", top_preview);

    gettimeofday(&time, NULL);  //END-TIME
    totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);
    std::cout << "ENet time = " << totalTime << " ms" << std::endl;

    pthread_barrier_wait(&enet_done);
    cv::waitKey(1);
    pthread_barrier_wait(&ipm_done);
  }
}

void *ipmThread(void *arg)
{
  ipm mapper;

  while(true)
  {
    pthread_barrier_wait(&enet_done);

    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
    
    // Grab latest enet frame
    // pthread_mutex_lock(&enet_locker);
    // cv::Mat latest_enet = enet_frame.clone();
    // pthread_mutex_unlock(&enet_locker);


    // Ensure enet_frame is populated before processing
    // if(latest_enet.empty())
        // continue;

    // Apply IPM
    // pthread_mutex_lock(&ipm_locker);

    cv::resize(enet_frame, enet_frame, cv::Size(frameWidth, frameHeight));
    // Remove lens distortion
    cv::resize(enet_frame, enet_frame, cv::Size(1920, 1080));
    enet_frame = mapper.removeDistortion(enet_frame);
    // Apply IPM
    ipm_frame = mapper.mapPerspective(enet_frame);
    // pthread_mutex_unlock(&ipm_locker);
    // bottom_preview = ipm_frame.clone();

    // ipm_frame = latest_enet.clone();

    gettimeofday(&time, NULL);  //END-TIME
    totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);
    std::cout << "IPM time = " << totalTime << " ms" << std::endl;

    pthread_barrier_wait(&ipm_done);
    pthread_barrier_wait(&pc_done);
  }
}


void modeCallback(const std_msgs::Float32::ConstPtr& msg)
{
  mode = msg->data;
}

int main(int argc, char** argv) 
{

  // ROS node initialization
  ros::init(argc, argv, "safezone");
  ros::NodeHandle n;
  ros::Publisher safe_pub = n.advertise<PointCloudT>("safezone_pc", 1);
  ros::Publisher unsafe_pub = n.advertise<PointCloudT>("unsafezone_pc", 1);
  ros::Subscriber mode_sub = n.subscribe("apm_safemode", 1, modeCallback);

  // Initialize frame mutexes
  pthread_mutex_init(&frame_locker, NULL);
  pthread_mutex_init(&enet_locker, NULL);
  pthread_mutex_init(&ipm_locker, NULL);

  pthread_barrier_init(&camera_done, NULL, 2);
  pthread_barrier_init(&enet_done, NULL, 3);
  pthread_barrier_init(&ipm_done, NULL, 3);
  pthread_barrier_init(&pc_done, NULL, 2);


  // Declare threads
  pthread_t frame_t;
  pthread_t enet_t;
  pthread_t ipm_t;

  // Create threads
  pthread_create(&frame_t, NULL, cameraThread, NULL);
  pthread_create(&enet_t, NULL, enetThread, NULL);
  pthread_create(&ipm_t, NULL, ipmThread, NULL);

  // Publish latest ipm_frame as a PointCloud
  // TODO: Should be in its own thread
  while (true)
  {
    // Measure time differences between pointcloud publishes
    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long publishTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    // Wait for IPM thread to finish
    pthread_barrier_wait(&ipm_done);
    
    gettimeofday(&time, NULL); // Start Time
    long pcTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
    
    cv::Mat latest_ipm;
    // Retrieve latest ipm_frame
    // pthread_mutex_lock(&ipm_locker);
    latest_ipm = ipm_frame.clone();
    // pthread_mutex_unlock(&ipm_locker);

    if(latest_ipm.empty())
        continue;

    
    cv::Mat cloud_frame;
    cv::resize(latest_ipm, cloud_frame, cv::Size(pcwidth, pcheight));
    // std::cout << "Grabbing latest ipm_frame" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr unsafe_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    safe_cloud->header.frame_id = "/camera";
    safe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    unsafe_cloud->header.frame_id = "/camera";
    unsafe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    // Construct pointcloud from perspective mapped image

    for (int r = 0; r < pcheight; ++r)
    {
      for (int c = 0; c < pcwidth; ++c)
      {

        if (mode == 0) // Roads safe
        {
          if(cloud_frame.at<cv::Vec3b>(r,c)[0] != 0 || cloud_frame.at<cv::Vec3b>(r,c)[2] != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            unsafe_cloud->points.push_back(p);
          }
          else if (cloud_frame.at<cv::Vec3b>(r,c)[1] == 255) // Green mask, road
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            safe_cloud->points.push_back(p);
          }
        }

        else if (mode == 1) // Sidewalks safe
        {
          if(cloud_frame.at<cv::Vec3b>(r,c)[1] != 0 || cloud_frame.at<cv::Vec3b>(r,c)[2] != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            unsafe_cloud->points.push_back(p);
          }
          else if (cloud_frame.at<cv::Vec3b>(r,c)[0] == 255) // Blue mask, sidewalk
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            safe_cloud->points.push_back(p);
          }
        }

        else // Both roads and sidewalks safe
        {
          if(cloud_frame.at<cv::Vec3b>(r,c)[2] != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            unsafe_cloud->points.push_back(p);
          }
          else // Blue mask, sidewalk
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * 3.66;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * 7.315;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = cloud_frame.at<cv::Vec3b>(r,c)[2];
            p.g = cloud_frame.at<cv::Vec3b>(r,c)[1];
            p.b = cloud_frame.at<cv::Vec3b>(r,c)[0];
            safe_cloud->points.push_back(p);
          }
        }

      }
    }
    safe_cloud->width = (int) safe_cloud->points.size();
    safe_cloud->height = 1;

    unsafe_cloud->width = (int) unsafe_cloud->points.size();
    unsafe_cloud->height = 1;

    safe_pub.publish(safe_cloud);
    unsafe_pub.publish(unsafe_cloud);

    // TODO: Make thread safe
    // cv::imshow("cloud frame", cloud_frame);
    // cv::Mat top_preview, bottom_preview, preview;

    // // Overlay segmentation mask onto original image
    // cv::addWeighted(frame, 1, enet_frame, 0.7, 0, overlay);

    // cv::hconcat(frame, enet_frame, top_preview);
    // cv::hconcat(overlay, ipm_frame, bottom_preview);
    // cv::vconcat(top_preview, bottom_preview, preview);

    // // cv::imshow("top_preview", top_preview);
    // cv::resize(preview, preview, cv::Size(), 0.80, 0.80);
    // cv::imshow("preview", preview);


    // cv::imshow("Original Frame", img);
    // cv::imshow("ENet Output Frame", enet_frame);
    // cv::imshow("Perspective Mapped Frame", output_frame);

    // cv::waitKey(1);
    gettimeofday(&time, NULL);  //END-TIME
    pcTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - pcTime);
    publishTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - publishTime);
    std::cout << "PC time = " << pcTime << " ms" << std::endl;
    std::cout << "Publish time = " << publishTime << " ms" << std::endl;

    ros::spinOnce(); // TODO: Make new thread?
    pthread_barrier_wait(&pc_done);
  }
}
