/*
 *
 *  Created on: Jan 21, 2017
 *      Author: Timo Sämann
 *    Modified: Robert Relyea rer3378@rit.edu
 *
 *  The basis for the creation of this script was the classification.cpp example (caffe/examples/cpp_classification/classification.cpp)
 *
 *  This script visualize the semantic segmentation for your input image.
 *
 *  To compile this script you can use a IDE like Eclipse. To include Caffe and OpenCV in Eclipse please refer to
 *  http://tzutalin.blogspot.de/2015/05/caffe-on-ubuntu-eclipse-cc.html
 *  and http://rodrigoberriel.com/2014/10/using-opencv-3-0-0-with-eclipse/ , respectively
 *
 *
 */

#include "include/classifier.hpp"

Classifier::Classifier(const string& model_file,
                       const string& trained_file,
                       const string& LUT_file) {

  label_colors = cv::imread(LUT_file,1);
  cv::cvtColor(label_colors, label_colors, CV_RGB2BGR);

  Caffe::set_mode(Caffe::GPU);

  /* Load the network. */
  net_.reset(new Net<float>(model_file, TEST));
  net_->CopyTrainedLayersFrom(trained_file);

  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  net_->Reshape();
}


cv::Mat Classifier::Predict(const cv::Mat& img){  // } string LUT_file) {
  // Blob<float>* input_layer = net_->input_blobs()[0];
  // input_layer->Reshape(1, num_channels_,
  //                      input_geometry_.height, input_geometry_.width);
  // /* Forward dimension change to all layers. */
  // net_->Reshape();

  std::vector<cv::Mat> input_channels;

  struct timeval time;
  
  gettimeofday(&time, NULL); // Start Time
  long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  // std::cout << "Enter Prediction = " << totalTime << " ms" << std::endl;
  WrapInputLayer(&input_channels);
  gettimeofday(&time, NULL);  //END-TIME
  totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);

  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  // std::cout << "Input layer wrapper " << totalTime << " ms" << std::endl;


  gettimeofday(&time, NULL); // Start Time
  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  Preprocess(img, &input_channels);
  gettimeofday(&time, NULL);  //END-TIME
  totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);
  // std::cout << "Preprocess " << totalTime << " ms" << std::endl;

  gettimeofday(&time, NULL); // Start Time
  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  net_->Forward();
  gettimeofday(&time, NULL);  //END-TIME
  totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);

  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  // std::cout << "Processing time = " << totalTime << " ms" << std::endl;

  //std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
  //std::cout << "Processing time = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/1000000.0 << " sec" <<std::endl; //Just for time measurement


  gettimeofday(&time, NULL); // Start Time
  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

  /* Copy the output layer to a std::vector */
  Blob<float>* output_layer = net_->output_blobs()[0];

  int width = output_layer->width();
  int height = output_layer->height(); 
  int channels = output_layer->channels();
  int num = output_layer->num();

  cv::Mat prediction_map (channels, width*height, CV_32FC1, const_cast<float *>(output_layer->cpu_data()));
  prediction_map.convertTo(prediction_map, CV_8UC1);
  prediction_map = prediction_map.reshape(1, 360);

  totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);

  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  // std::cout << "Postprocessing time = " << totalTime << " ms" << std::endl;

  gettimeofday(&time, NULL); // Start Time
  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

  cv::Mat vis = Visualization(prediction_map);

  totalTime = (((time.tv_sec * 1000) + (time.tv_usec / 1000)) - totalTime);

  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  // std::cout << "Visualization time = " << totalTime << " ms" << std::endl;

  gettimeofday(&time, NULL); // Start Time
  totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);
  // std::cout << "Exit Prediction = " << totalTime << " ms" << std::endl;

  return vis;
}


cv::Mat Classifier::Visualization(cv::Mat prediction_map) {

  cv::Mat output_frame;

  cv::cvtColor(prediction_map.clone(), prediction_map, CV_GRAY2BGR);
  // cv::Mat output_image;
  LUT(prediction_map, label_colors, output_frame);

  return output_frame;

  // cv::imshow( "Display window", output_image);
  // cv::waitKey(1);
}


/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void Classifier::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void Classifier::Preprocess(const cv::Mat& img,
                            std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_float, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}
