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
#include <cstdlib>

// Safezone includes
#include "include/classifier.hpp"
#include "include/ipm.hpp"

// Define for ROS topic vs. /dev/video0
#define CAMERA_VIA_ROS

typedef pcl::PointCloud <pcl::PointXYZRGB> PointCloudT;

// Pointcloud publish params
typedef enum pc_params {
    pcwidth = 80,
    pcheight = 45,
} pc_params;

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


#ifndef CAMERA_VIA_ROS
// Publisher to publish camera data
image_transport::Publisher image_pub;
#else

#include <sensor_msgs/Image.h>
#include <condition_variable>

cv::Mat new_frame;
std::condition_variable cond_var;
//sensor_msgs::ImageConstPtr new_image = nullptr;
bool new_image = false;
std::mutex m;

// Function for reading from topic
//void handleImage(const sensor_msgs::ImageConstPtr &img_msg,
//                 const sensor_msgs::CameraInfoConstPtr &info_msg) {
void handleImage(const sensor_msgs::ImageConstPtr &img_msg) {
    try {
        new_image = (img_msg != nullptr);
        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(img_msg);
//        new_frame = cv_image->image.clone();
        cv::resize(cv_image->image, new_frame, cv::Size(1920, 1080));
        if (new_frame.empty())
            new_image = false;
        else
            cond_var.notify_one();
    } catch (cv_bridge::Exception &e) {
        new_image = false;
        ROS_ERROR("Unable to convert %s image to bgr8", img_msg->encoding.c_str());
    }
}

#endif

// pthread_barrier_t camera_done;
// pthread_barrier_t enet_done;
// pthread_barrier_t ipm_done;
// pthread_barrier_t pc_done;

/* Function for camera capture thread
*/
void *cameraThread(void *arg) {
    // Video capture initialization
    // cv::VideoCapture cap("/home/apm/Videos/Webcam/2018-04-02-171452.webm");
    // cv::VideoCapture cap("/home/apm/Videos/Webcam/2018-04-20-121559.webm");
#ifndef CAMERA_VIA_ROS
    cv::VideoCapture cap("/dev/video0");
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
#endif

    while (ros::ok()) {
        struct timeval time;
        gettimeofday(&time, NULL); // Start Time
        long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

        // Grab new frame
        cv::Mat new_frame_local;
#ifndef CAMERA_VIA_ROS
        cap >> new_frame_local;
#else
        std::unique_lock <std::mutex> lk(m);
//        cond_var.wait(lk, [] { return new_image != nullptr; });
        cond_var.wait(lk, [] { return new_image; });
#endif
        // Resize new frame
//        std::cout << "Resize1: " << new_frame.total() << std::endl;
        if (!new_frame.size().height || !new_frame.size().width)
            std::cout << new_frame.size().height << ',' << new_frame.size().width << std::endl;
        cv::resize(new_frame, new_frame_local, cv::Size(frameWidth, frameHeight));

#ifndef CAMERA_VIA_ROS
        // Create the ROS message to broadcast images
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", new_frame_local).toImageMsg();
        image_pub.publish(msg);
#endif

        // Copy new frame to shared resource
        pthread_mutex_lock(&frame_locker);
        frame = new_frame_local.clone();
        pthread_mutex_unlock(&frame_locker);

        // cv::imshow("Preview", new_frame_local);
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


void *enetThread(void *arg) {
    // ENet initialization
    std::string tt_root = std::getenv("TT_ROOT");
    std::string model_file = tt_root + "/tt_core/ENet/final_model_weights/bn_conv_merged_model.prototxt";
    std::string trained_file = tt_root + "/tt_core/ENet/final_model_weights/bn_conv_merged_weights.caffemodel";
    std::string LUT_file = tt_root + "/tt_core/safezone_lut/safezone.png";

    Classifier classifier(model_file, trained_file, LUT_file);

    while (ros::ok()) {
        // Grab latest frame
        pthread_mutex_lock(&frame_locker);
        cv::Mat latest_frame = frame.clone();
        pthread_mutex_unlock(&frame_locker);

        // Ensure frame is populated before processing
        if (latest_frame.empty()) {
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

void *ipmThread(void *arg) {
    ipm mapper;

    while (ros::ok()) {
        struct timeval time;
        gettimeofday(&time, NULL); // Start Time
        long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

        // Grab latest enet frame
        pthread_mutex_lock(&enet_locker);
        cv::Mat latest_enet = enet_frame.clone();
        pthread_mutex_unlock(&enet_locker);

        // Ensure enet_frame is populated before processing
        if (latest_enet.empty()) {
            sleep(0.1);
            continue;
        }

        // Apply IPM
        cv::Mat new_ipm_frame;
        // Resize for distortion removal
        // TODO: calibrate camera at lower res?
        //       - Not a whole lot of perf to gain with faster IPM... (~10ms)
//        std::cout << "Resize2: " << latest_enet.total() << std::endl;
        cv::resize(latest_enet, latest_enet, cv::Size(frameWidth, frameHeight));
        // Remove lens distortion
//        std::cout << "Resize3: " << latest_enet.total() << std::endl;
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

void *previewThread(void *arg) {
    cv::Mat latest_frame, latest_enet, latest_ipm, overlay;

    while (ros::ok()) {
        // Acquire latest frames
        pthread_mutex_lock(&frame_locker);
        latest_frame = frame.clone();
        pthread_mutex_unlock(&frame_locker);

        if (latest_frame.empty()) {
            sleep(0.1);
            continue;
        }

        pthread_mutex_lock(&enet_locker);
        latest_enet = enet_frame.clone();
        pthread_mutex_unlock(&enet_locker);

        if (latest_enet.empty()) {
            sleep(0.1);
            continue;
        }

        pthread_mutex_lock(&ipm_locker);
        latest_ipm = ipm_frame.clone();
        pthread_mutex_unlock(&ipm_locker);

        if (latest_ipm.empty()) {
            sleep(0.1);
            continue;
        }

        // Create visualization
        cv::Mat top_preview, bottom_preview, preview;

        // Overlay segmentation mask onto original image
        //std::cout << "Width : " << latest_frame.cols << std::endl;
        //std::cout << "Height: " << latest_frame.rows << std::endl;
        //std::cout << "Width : " << latest_enet.cols << std::endl;
        //std::cout << "Height: " << latest_enet.rows << std::endl;
        //std::cout << "Width : " << overlay.cols << std::endl;
        //std::cout << "Height: " << overlay.rows << std::endl;
        cv::addWeighted(latest_frame, 1, latest_enet, 0.7, 0, overlay); // This is where things break

        cv::copyMakeBorder(latest_frame, latest_frame, 0, 5, 0, 5, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::copyMakeBorder(latest_enet, latest_enet, 0, 5, 5, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::copyMakeBorder(overlay, overlay, 5, 0, 0, 5, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::copyMakeBorder(latest_ipm, latest_ipm, 5, 0, 5, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        cv::putText(latest_frame, "Camera Feed", cvPoint(30, 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 0), 1, CV_AA);
        cv::putText(latest_enet, "Road/Sidewalk Detection", cvPoint(30, 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 0), 1, CV_AA);
        cv::putText(latest_ipm, "Bird's-eye-view", cvPoint(30, 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 0), 1, CV_AA);
        cv::putText(overlay, "Overlay", cvPoint(30, 30),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 0), 1, CV_AA);


        cv::hconcat(latest_frame, latest_enet, top_preview);
        cv::hconcat(overlay, latest_ipm, bottom_preview);
        cv::vconcat(top_preview, bottom_preview, preview);

        // cv::imshow("top_preview", top_preview);
//        std::cout << "Resize4: " << preview.total() << std::endl;
        cv::resize(preview, preview, cv::Size(), 0.80, 0.80);
        cv::imshow("preview", preview);


        // cv::imshow("Original Frame", img);
        // cv::imshow("ENet Output Frame", latest_enet);
        // cv::imshow("Perspective Mapped Frame", output_frame);

        cv::waitKey(1);
    }
}

void modeCallback(const std_msgs::Float32::ConstPtr &msg) {
    mode = msg->data;
}

int main(int argc, char **argv) {

    // ROS node initialization
    ros::init(argc, argv, "safezone");
    ros::NodeHandle n;
    ros::Publisher safe_pub = n.advertise<PointCloudT>("safezone_pc", 1);
    ros::Publisher unsafe_pub = n.advertise<PointCloudT>("unsafezone_pc", 1);
    ros::Subscriber mode_sub = n.subscribe("apm_safemode", 1, modeCallback);

#ifndef CAMERA_VIA_ROS
    // ROS nodes for image publish
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("camera/image", 1);
#else
    image_transport::ImageTransport it(n);
//    image_transport::CameraSubscriber it_sub;
//    it_sub = it.subscribeCamera("/camera/image", 1, handleImage);
    image_transport::Subscriber it_sub = it.subscribe("camera/image", 1, &handleImage);
#endif

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
    while (ros::ok()) {
        ros::spinOnce();  // I swear to god if putting this at the top of this loop fixes my issue I will jump off Gleason
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

        if (latest_ipm.empty()) {
            // May kill perf, combine this thread with IPM?
            sleep(0.01);
            continue;
        }

        // Edge Detection
        cv::Mat cloud_frame;
//        std::cout << "Resize5: " << latest_ipm.total() << std::endl;
        cv::resize(latest_ipm, cloud_frame, cv::Size(pcwidth, pcheight));

        // Pad Bottom
        cv::Rect roi(0, 0, cloud_frame.cols, cloud_frame.rows);
        cv::copyMakeBorder(cloud_frame, cloud_frame, 0, 3, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        // Split into BRG channels
        std::vector <cv::Mat> cloud_frame_spl;
        cv::split(cloud_frame, cloud_frame_spl);

        cv::Mat unsafe_mask;
        if (mode == 0) // Road safe
        {
            // cloud_frame_spl[0].copyTo(unsafe_mask, cloud_frame_spl[2]);
            cv::add(cloud_frame_spl[0], cloud_frame_spl[2], unsafe_mask);
        } else if (mode == 1) // Sidewalk safe
        {
            // cloud_frame_spl[1].copyTo(unsafe_mask, cloud_frame_spl[2]);
            cv::add(cloud_frame_spl[1], cloud_frame_spl[2], unsafe_mask);
        } else // Road + Sidewalk safe
        {
            unsafe_mask = cloud_frame_spl[2].clone();
        }

        // Run edge detection
        cv::Canny(unsafe_mask.clone(), unsafe_mask, 50, 150, 3);
        unsafe_mask.convertTo(unsafe_mask, CV_8U);
        unsafe_mask = unsafe_mask(roi);

        // cv::imwrite("/home/apm/tt_core/safezone_lut/unsafe_mask.png", unsafe_mask);

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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr safe_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr unsafe_cloud(new pcl::PointCloud <pcl::PointXYZRGB>);

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

        for (int r = 0; r < pcheight; ++r) {
            for (int c = 0; c < pcwidth; ++c) {
                if (unsafe_mask.at<uchar>(r, c) != 0) {
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

        // ros::spinOnce(); // TODO: Make new thread?
        // pthread_barrier_wait(&pc_done);
    }
    pthread_exit(NULL);
}
