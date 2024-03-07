// Copyright (c) 2023，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <fstream>
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


class RosSubNode{
public:
  RosSubNode() : 
    is_running_(true),
    image_queue_limit_(1),
    imu_queue_limit_(800),
    image_frame_("default_camera"){

    ros::NodeHandle nh("~");
    nh.param("image_topic", sub_image_topic_, std::string("/camera/color/image_raw"));
    nh.param("imu_topic", sub_imu_topic_, std::string("/camera/imu"));
    nh.param("sample_gap", sample_gap_, 2);
    nh.param("img_que_limit", image_queue_limit_, 10);
    nh.param("imu_que_limit", imu_queue_limit_, 400);

    ROS_INFO("image_topic:  %s", sub_image_topic_.c_str());
    ROS_INFO("imu_topic: %s", sub_imu_topic_.c_str());
    ROS_INFO("sample_gap: %d", sample_gap_);
    ROS_INFO("img_que_limit: %d", image_queue_limit_);
    ROS_INFO("imu_que_limit: %d", imu_queue_limit_);

    sub_imu_ = nh.subscribe(sub_imu_topic_, 1000, &RosSubNode::CollectImu, this);
    sub_img_ = nh.subscribe(sub_image_topic_, 30, &RosSubNode::CollectImage, this);
  }

  void SendImu(float *imu_data, uint64_t timestamp) {
    auto *imu_copy = new float[6];
    std::shared_ptr<float> imu_copy_ptr(imu_copy, [](float *p) {
      delete []p;
    });
    std::memcpy(imu_copy, imu_data, sizeof(float) * 6);
    {
      std::lock_guard<std::mutex> lck(imu_mutex_);
      imu_queue_.emplace_back(std::make_pair(timestamp, imu_copy_ptr));
      if (imu_queue_limit_ > 0 && imu_queue_.size() > (uint64_t)imu_queue_limit_) {
        ROS_WARN("imu queue size is over: %d" , imu_queue_limit_);
        imu_queue_.pop_front();
      }
    }
    imu_cv_.notify_one();
  }

  void ShutDown() {
    if (!is_running_) {
      return;
    }
    is_running_ = false;
    imu_cv_.notify_all();
    image_cv_.notify_all();
  }

  ~RosSubNode() {
    ShutDown();
  }

  int GetImage(uint64 &ts, cv::Mat &image, uint& queue_size) {
    std::unique_lock<std::mutex> lck(image_mutex_);
    image_cv_.wait(lck,[&]()
    { return !image_queue_.empty() || !is_running_;});
    if (!is_running_) return -1;
    ts = image_queue_.begin()->first;
    image = image_queue_.begin()->second;
    image_queue_.erase(image_queue_.begin());
    queue_size = image_queue_.size();
    return 0;
  }

  int GetImusByTimeStamp(uint64 ts,
          std::deque<std::pair<uint64, std::shared_ptr<float>>> &data) {
    std::unique_lock<std::mutex> lck(imu_mutex_);
    imu_cv_.wait(lck, [&]() {
      return (!imu_queue_.empty() && ts < imu_queue_.back().first) || !is_running_;});
    if (!is_running_) return -1;
    auto current_data = imu_queue_.begin();
    while (ts >= current_data->first && current_data != imu_queue_.end()) {
      ++current_data;
    }
    data.assign(imu_queue_.begin(), current_data);
    imu_queue_.erase(imu_queue_.begin(), current_data);
    return 0;
  }

  void CollectImu(const sensor_msgs::Imu::ConstPtr& imu) {
    if (!is_running_) return;
    uint64_t timestamp =
            imu->header.stamp.sec * 1e9 + imu->header.stamp.nsec;
    // static uint64_t last_ts = timestamp;
    // if (timestamp - last_ts > 3502912) {
    //   ROS_WARN("find imu gap! imu gap: %ld\n", timestamp - last_ts);
    // }
    // last_ts = timestamp;
    float imu_data[9];
    imu_data[0] = imu->linear_acceleration.x;
    imu_data[1] = imu->linear_acceleration.y;
    imu_data[2] = imu->linear_acceleration.z;
    imu_data[3] = imu->angular_velocity.x;
    imu_data[4] = imu->angular_velocity.y;
    imu_data[5] = imu->angular_velocity.z;
    SendImu(imu_data, timestamp);
  }

  // void CollectCompressImage(
  //         const sensor_msgs::CompressedImage::SharedPtr img) {
  //   if (!is_running_) return;
  //   uint64_t timestamp =
  //           img->header.stamp.sec * 1e9 + img->header.stamp.nanosec;
  //   static uint64_t last_ts = timestamp;
  //   if (timestamp - last_ts > 45029120) {
  //   //  std::cout << "find image gap! image gap: " << timestamp - last_ts << std::endl;
  //   }
  //   last_ts = timestamp;
  //   cv::Mat image_mat;
  //   try {
  //     cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(
  //             img, sensor_msgs::image_encodings::MONO8);
  //     image_mat = cv_ptr_compressed->image;
  //   } catch (cv_bridge::Exception &e) {
  //     ROS_WARN("convert CompressImage to mono8 failed");
  //     return;
  //   }
  //   {
  //     std::lock_guard<std::mutex> lck(image_mutex_);
  //     image_queue_[timestamp] = image_mat.clone();
  //     if (image_queue_limit_ > 0 && image_queue_.size() > image_queue_limit_) {
  //       image_queue_.erase(image_queue_.begin());
  //     }
  //   }
  //   image_cv_.notify_one();
  // }

  std::string GetImageFrame() {
    {
      std::lock_guard<std::mutex> lck(frame_mtx_);
      return image_frame_;
    }
  }

  void CollectImage(const sensor_msgs::ImagePtr& img) {
    if (!is_running_) return;
    cv::Mat image;
    uint64_t timestamp =
            img->header.stamp.sec * 1e9 + img->header.stamp.nsec;
    static int d = 0;
    if (++d != sample_gap_) {
      return;
    }
    d = 0;
    {
      std::lock_guard<std::mutex> lck(frame_mtx_);
      image_frame_ = img->header.frame_id;
    }
    try {
      const std::string &encoding = img->encoding;
      if (encoding == "nv12") {
        //  only reserve the part Y
        image = cv::Mat(img->height, img->width,
                CV_8UC1, img->data.data());
      } else {
        image = cv_bridge::toCvShare(img)->image;
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("receive unknow image: %s", e.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lck(image_mutex_);
      image_queue_[timestamp] = image.clone();
      if (image_queue_limit_ > 0 && image_queue_.size() > (uint64_t)image_queue_limit_) {
        ROS_WARN("image queue size is over %d",image_queue_limit_);
        image_queue_.erase(image_queue_.begin());
      }
    }
    image_cv_.notify_one();
  }

  std::atomic_bool is_running_;
  std::map<uint64, cv::Mat> image_queue_;
  int32_t sample_gap_ = 2;
  int32_t image_queue_limit_, imu_queue_limit_;
  std::mutex image_mutex_;
  std::condition_variable image_cv_;
  std::mutex frame_mtx_;
  std::string image_frame_;

  ros::Subscriber sub_img_;
  ros::Subscriber sub_imu_;

  std::deque<std::pair<uint64, std::shared_ptr<float>>> imu_queue_;
  std::mutex imu_mutex_, imu_pub_mutex_;
  std::condition_variable imu_cv_, imu_pub_cv_;

  std::string sub_image_topic_, sub_compressed_image_topic_;
  std::string sub_imu_topic_;
};

