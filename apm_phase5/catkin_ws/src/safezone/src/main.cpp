/*
 *
 *  Created on: Feb 5, 2018
 *      Author: Robert Relyea rer3378@rit.edu
 *
 */

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pthread.h>
#include <string>

// Safezone includes
#include "include/classifier.hpp"
#include "include/ipm.hpp"

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

// shared_ptr<Net<float> > net_;
// cv::Size input_geometry_;
// int num_channels_;

// Mat variables for storing images
// cv::Mat img; // Create mat to hold new frames
// cv::Mat enet_frame, output_frame;

// Mats for storing image operations
// frame:       New frame captured by camera
// enet_frame:  Resulting segmented camera frame
// ipm_frame:   Resulting perspective mapped segmented frame
cv::Mat frame, enet_frame, colored_enet_frame, ipm_frame;

// Mats for generating preview image
cv::Mat top_preview, bottom_preview, preview;

// Mutexes for locking intermediate image operation matricies
pthread_mutex_t frame_locker;
pthread_mutex_t enet_locker;
pthread_mutex_t colored_enet_locker;
pthread_mutex_t ipm_locker;
pthread_mutex_t preview_locker;


// Publisher to publish camera data
image_transport::Publisher image_pub;

// pthread_barrier_t camera_done;
// pthread_barrier_t enet_done;
// pthread_barrier_t ipm_done;
// pthread_barrier_t pc_done;

/* Function for camera capture thread
*/
void *cameraThread(void *arg)
{
  // Video capture initialization
  // cv::VideoCapture cap("/home/apm/Videos/Webcam/2018-04-02-171452.webm");
  // cv::VideoCapture cap("/home/apm/Videos/Webcam/2018-04-20-121559.webm");
  cv::VideoCapture cap("/dev/video0");
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
  cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));

  while(true)
  {
    
    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    // Grab new frame
    cv::Mat new_frame;
    cap >> new_frame;  

    // Resize new frame
    cv::resize(new_frame, new_frame, cv::Size(frameWidth, frameHeight));

    // Create the ROS message to broadcast images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", new_frame).toImageMsg();
    image_pub.publish(msg);

    // Copy new frame to shared resource
    pthread_mutex_lock(&frame_locker);
    frame = new_frame.clone();
    pthread_mutex_unlock(&frame_locker);

    // cv::imshow("Preview", new_frame);
    // cv::waitKey(1);

    gettimeofday(&time, NULL);  //END-TIME
    totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);
    // std::cout << "Camera time = " << totalTime << " ms" << std::endl;

    // cv::imshow("frame", frame);
    // cv::waitKey(1);

    // pthread_barrier_wait(&camera_done);
    // pthread_barrier_wait(&enet_done);
  }
}


void *enetThread(void *arg)
{
  // ENet initialization
  std::string model_file   = "/home/rosmaster/apm/apm_phase5/ENet/final_model_weights/bn_conv_merged_model.prototxt";
  std::string trained_file = "/home/rosmaster/apm/apm_phase5/ENet/final_model_weights/bn_conv_merged_weights.caffemodel";
  std::string LUT_file = "/home/rosmaster/apm/apm_phase5/safezone.png";

  Classifier classifier(model_file, trained_file, LUT_file);

  while(true)
  {


    // Grab latest frame
    pthread_mutex_lock(&frame_locker);
    cv::Mat latest_frame = frame.clone();
    pthread_mutex_unlock(&frame_locker);

    // Ensure frame is populated before processing
    if(latest_frame.empty())
    { 
      // Sleep for a little while camera initializes
      sleep(0.1);
      continue;
    }

    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long enetTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
    // std::cout << "ENet call time = " << totalTime << " ms" << std::endl;

    // Pass latest frame through ENet
    cv::Mat new_enet_frame;
    new_enet_frame = classifier.Predict(latest_frame);
    // cv::imwrite("/home/apm/test.png", prediction_map);


    gettimeofday(&time, NULL);  //END-TIME
    enetTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - enetTime);
    std::cout << "ENet time = " << enetTime << " ms" << std::endl;
    std::cout << "ENet FPS = " << 1000 / enetTime << std::endl;

    // Run canny edge detection
    // cv::Mat new_enet_frame_edges = new_enet_frame.clone();
    // cv::Canny(new_enet_frame, new_enet_frame_edges, 50, 150, 3);

    // std::cout << "Recoloring" << std::endl;
    // new_enet_frame_edges.convertTo(new_enet_frame_edges, CV_8UC3);

    std::cout << "Done" << std::endl;

    // Copy new enet frame to shared resource
    pthread_mutex_lock(&enet_locker);
    enet_frame = new_enet_frame.clone();
    pthread_mutex_unlock(&enet_locker);
  }
}

void *ipmThread(void *arg)
{
  ipm mapper;

  while(true)
  {
    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
    
    // Grab latest enet frame
    pthread_mutex_lock(&enet_locker);
    cv::Mat latest_enet = enet_frame.clone();
    pthread_mutex_unlock(&enet_locker);

    // Ensure enet_frame is populated before processing
    if(latest_enet.empty())
    {
      sleep(0.1);
      continue;
    }
   
    // Apply IPM
    cv::Mat new_ipm_frame;
    // Resize for distortion removal
    // TODO: calibrate camera at lower res?
    //       - Not a whole lot of perf to gain with faster IPM... (~10ms)
    cv::resize(latest_enet, latest_enet, cv::Size(frameWidth, frameHeight));
    // Remove lens distortion
    cv::resize(latest_enet, latest_enet, cv::Size(1920, 1080));
    latest_enet = mapper.removeDistortion(latest_enet);
    // Apply IPM
    new_ipm_frame = mapper.mapPerspective(latest_enet);

    // Copy new ipm frame to shared resource
    pthread_mutex_lock(&ipm_locker);
    ipm_frame = new_ipm_frame.clone();
    pthread_mutex_unlock(&ipm_locker);
    // bottom_preview = ipm_frame.clone();

    // ipm_frame = latest_enet.clone();

    gettimeofday(&time, NULL);  //END-TIME
    totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);
    // std::cout << "IPM time = " << totalTime << " ms" << std::endl;

    // pthread_barrier_wait(&ipm_done);
    // pthread_barrier_wait(&pc_done);
  }
}

void *previewThread(void *arg)
{
  cv::Mat latest_frame, latest_enet, latest_ipm, overlay;

  for(;;)
  {
    // Aqcuire latest frames
    pthread_mutex_lock(&frame_locker);
    latest_frame = frame.clone();
    pthread_mutex_unlock(&frame_locker);

    if(latest_frame.empty())
    {
      sleep(0.1);
      continue;
    }

    pthread_mutex_lock(&enet_locker);
    latest_enet = enet_frame.clone();
    pthread_mutex_unlock(&enet_locker);

    if(latest_enet.empty())
    {
      sleep(0.1);
      continue;
    }

    pthread_mutex_lock(&ipm_locker);
    latest_ipm = ipm_frame.clone();
    pthread_mutex_unlock(&ipm_locker);

    if(latest_ipm.empty())
    {
      sleep(0.1);
      continue;
    }

    // Create visualization
    cv::Mat top_preview, bottom_preview, preview;

    // Overlay segmentation mask onto original image
    cv::addWeighted(latest_frame, 1, latest_enet, 0.7, 0, overlay);
    
    cv::copyMakeBorder( latest_frame, latest_frame, 0, 5, 0, 5, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );
    cv::copyMakeBorder( latest_enet, latest_enet, 0, 5, 5, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );
    cv::copyMakeBorder( overlay, overlay, 5, 0, 0, 5, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );
    cv::copyMakeBorder( latest_ipm, latest_ipm, 5, 0, 5, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );

    cv::putText(latest_frame, "Camera Feed", cvPoint(30,30), 
    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
    cv::putText(latest_enet, "Road/Sidewalk Detection", cvPoint(30,30), 
    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
    cv::putText(latest_ipm, "Bird's-eye-view", cvPoint(30,30), 
    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
    cv::putText(overlay, "Overlay", cvPoint(30,30), 
    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);


    cv::hconcat(latest_frame, latest_enet, top_preview);
    cv::hconcat(overlay, latest_ipm, bottom_preview);
    cv::vconcat(top_preview, bottom_preview, preview);

    // cv::imshow("top_preview", top_preview);
    cv::resize(preview, preview, cv::Size(), 0.80, 0.80);
    cv::imshow("preview", preview);


    // cv::imshow("Original Frame", img);
    // cv::imshow("ENet Output Frame", latest_enet);
    // cv::imshow("Perspective Mapped Frame", output_frame);

    cv::waitKey(1);
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

  // ROS nodes for image publish
  image_transport::ImageTransport it(n);
  image_pub = it.advertise("camera/image",1);

  // Initialize frame mutexes
  pthread_mutex_init(&frame_locker, NULL);
  pthread_mutex_init(&enet_locker, NULL);
  pthread_mutex_init(&colored_enet_locker, NULL);
  pthread_mutex_init(&ipm_locker, NULL);

  // pthread_barrier_init(&camera_done, NULL, 2);
  // pthread_barrier_init(&enet_done, NULL, 3);
  // pthread_barrier_init(&ipm_done, NULL, 3);
  // pthread_barrier_init(&pc_done, NULL, 2);


  // Declare threads
  pthread_t frame_t;
  pthread_t enet_t;
  pthread_t ipm_t;
  pthread_t vis_t;

  // Create threads
  pthread_create(&frame_t, NULL, cameraThread, NULL);
  pthread_create(&enet_t, NULL, enetThread, NULL);
  pthread_create(&ipm_t, NULL, ipmThread, NULL);
  pthread_create(&vis_t, NULL, previewThread, NULL);

  // Publish latest ipm_frame as a PointCloud
  // TODO: Should be in its own thread
  while (true)
  {
    // Measure time differences between pointcloud publishes
    struct timeval time;
    gettimeofday(&time, NULL); // Start Time
    long publishTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    // Wait for IPM thread to finish
    // pthread_barrier_wait(&ipm_done);
    
    gettimeofday(&time, NULL); // Start Time
    long pcTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
    
    cv::Mat latest_ipm;
    // Retrieve latest ipm_frame
    pthread_mutex_lock(&ipm_locker);
    latest_ipm = ipm_frame.clone();
    pthread_mutex_unlock(&ipm_locker);

    if(latest_ipm.empty())
    {
      // May kill perf, combine this thread with IPM?
      sleep(0.01);
      continue;
    }
    
    // Edge Detection
    cv::Mat cloud_frame;
    cv::resize(latest_ipm, cloud_frame, cv::Size(pcwidth, pcheight));
    
    // Pad Bottom
    cv::Rect roi(0,0,cloud_frame.cols, cloud_frame.rows);
    cv::copyMakeBorder(cloud_frame, cloud_frame, 0, 3, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

    // Split into BRG channels
    std::vector<cv::Mat> cloud_frame_spl;
    cv::split(cloud_frame, cloud_frame_spl);

    cv::Mat unsafe_mask;
    if (mode == 0) // Road safe
    {
      // cloud_frame_spl[0].copyTo(unsafe_mask, cloud_frame_spl[2]);
      cv::add(cloud_frame_spl[0], cloud_frame_spl[2], unsafe_mask);
    }
    else if (mode == 1) // Sidewalk safe
    {
      // cloud_frame_spl[1].copyTo(unsafe_mask, cloud_frame_spl[2]);
      cv::add(cloud_frame_spl[1], cloud_frame_spl[2], unsafe_mask);
    }
    else // Road + Sidewalk safe
    {
      unsafe_mask = cloud_frame_spl[2].clone();
    }

    // Run edge detection
    cv::Canny(unsafe_mask.clone(), unsafe_mask, 50, 150, 3);
    unsafe_mask.convertTo(unsafe_mask, CV_8U);
    unsafe_mask = unsafe_mask(roi);

    // cv::imwrite("/home/apm/apm_phase5/unsafe_mask.png", unsafe_mask);

/*
    cv::Mat cloud_frame_b, cloud_frame_g, cloud_frame_r;
    cv::Canny(cloud_frame_spl[0], cloud_frame_b, 50, 150, 3);
    cv::Canny(cloud_frame_spl[1], cloud_frame_g, 50, 150, 3);
    cv::Canny(cloud_frame_spl[2], cloud_frame_r, 50, 150, 3);
    cloud_frame_b.convertTo(cloud_frame_b, CV_8U);
    cloud_frame_g.convertTo(cloud_frame_g, CV_8U);
    cloud_frame_r.convertTo(cloud_frame_r, CV_8U);
    cloud_frame_b = cloud_frame_b(roi);
    cloud_frame_g = cloud_frame_g(roi);
    cloud_frame_r = cloud_frame_r(roi);
*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr unsafe_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    safe_cloud->header.frame_id = "/camera";
    safe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    unsafe_cloud->header.frame_id = "/camera";
    unsafe_cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    // float measured_height = 3.66;
    // float measured_width = 7.315;

    // float measured_height = 7.38;
    float measured_height = 10.15;
    float measured_width = 19;

    // Construct pointcloud from perspective mapped image

    for (int r = 0; r < pcheight; ++r)
    {
      for (int c = 0; c < pcwidth; ++c)
      {
        if(unsafe_mask.at<uchar>(r,c) != 0)
        {
          pcl::PointXYZRGB p;
          //The coordinate of the point is taken from the depth map
          //Y and Z  taken negative to immediately visualize the cloud in the right way
          p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
          p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
          p.z = 0;

          // Color point for unsafe zones (BGR)
          p.r = 255;
          unsafe_cloud->points.push_back(p);
        }
        else
        {
          pcl::PointXYZRGB p;
          //The coordinate of the point is taken from the depth map
          //Y and Z  taken negative to immediately visualize the cloud in the right way
          p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
          p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
          p.z = 0;

          // Color point for unsafe zones (BGR)
          p.b = 255;
          safe_cloud->points.push_back(p);
        }
      }
    }
    /*
    for (int r = 0; r < pcheight; ++r)
    {
      for (int c = 0; c < pcwidth; ++c)
      {

        if (mode == 0) // Roads safe
        {
          if(cloud_frame_b.at<uchar>(r,c) != 0 || cloud_frame_r.at<uchar>(r,c) != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = 255;
            unsafe_cloud->points.push_back(p);
          }
          else
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.b = 255;
            safe_cloud->points.push_back(p);
          }
        }

        if (mode == 1) // sidewalks safe
        {
          if(cloud_frame_g.at<uchar>(r,c) != 0 || cloud_frame_r.at<uchar>(r,c) != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = 255;
            unsafe_cloud->points.push_back(p);
          }
          else
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.b = 255;
            safe_cloud->points.push_back(p);
          }
        }

        else // Both roads and sidewalks safe
        {
          if(cloud_frame_r.at<uchar>(r,c) != 0)
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.r = 255;
            unsafe_cloud->points.push_back(p);
          }
          else // Blue mask, sidewalk
          {
            pcl::PointXYZRGB p;
            //The coordinate of the point is taken from the depth map
            //Y and Z  taken negative to immediately visualize the cloud in the right way
            p.x = ((r - pcheight / 2.0) / (float)pcheight) * measured_height;
            p.y = ((c - pcwidth / 2.0) / (float)pcwidth) * measured_width;
            p.z = 0;

            // Color point for unsafe zones (BGR)
            p.b = 255;
            safe_cloud->points.push_back(p);
          }
        }

      }
    }
    */
    safe_cloud->width = (int) safe_cloud->points.size();
    safe_cloud->height = 1;

    unsafe_cloud->width = (int) unsafe_cloud->points.size();
    unsafe_cloud->height = 1;

    safe_pub.publish(safe_cloud);
    unsafe_pub.publish(unsafe_cloud);

    gettimeofday(&time, NULL);  //END-TIME
    pcTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - pcTime);
    publishTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - publishTime);
    // std::cout << "PC time = " << pcTime << " ms" << std::endl;
    // std::cout << "Publish time = " << publishTime << " ms" << std::endl;

    ros::spinOnce(); // TODO: Make new thread?
    // pthread_barrier_wait(&pc_done);
  }
}
