/*
 *
 *  Created on: Mar 15, 2018
 *      Author: Robert Relyea rer3378@rit.edu
 *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std; // gross
#include <opencv2/gpu/gpu.hpp>


int main(int argc, char** argv) 
{
  // init GPU
  cv::gpu::setDevice(0);
  cv::gpu::GpuMat image_gpu, result_gpu;

  // imread with CPU
  cv::Mat image, result;
  image = cv::imread("../test.png", CV_LOAD_IMAGE_COLOR);   // Read the file
  std::cout << "Read image" << std::endl;

  // upload to GPU
  image_gpu.upload(image);
  std::cout << "Uploaded image to GPU" << std::endl;

  // Resize image on GPU
  cv::gpu::resize(image_gpu, result_gpu, cv::Size(), .25, 0.25, CV_INTER_AREA);
  std::cout << "Resized image on GPU" << std::endl;
  // Download resized image
  result_gpu.download(result);
  std::cout << "Downloaded image from GPU" << std::endl;
  // Write output
  cv::imwrite("../output.png", result);
  std::cout << "Wrote output image" << std::endl;
}
