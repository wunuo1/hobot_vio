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

#include <fstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <strstream>
#include <ctime>
#include <iomanip>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"

#include "data_collection.h"
#include "HorizonVIO.h"

class ROSVisualizer{
  using ROS_Image = sensor_msgs::Image;
  using ROS_Path = nav_msgs::Path;
  using ROS_Odom = nav_msgs::Odometry;

public:
  ros::NodeHandle visualizer_nh;
  static std::shared_ptr<ROSVisualizer> GetInstance() {
    static auto instance = std::make_shared<ROSVisualizer>();
    return instance;
  }

  ROSVisualizer() :
  is_pose_updated_(false),
  is_img_updated_(false),
  is_running_(true) {

    ros::NodeHandle nh("~");
    nh.param("config_file_path", config_file_path_, std::string("/root/dev_ws/src/hobot_vio/config/realsenseD435i.yaml"));
    // image_publisher_ = std::make_shared<ros::Publisher>(visualizer_nh.advertise<ROS_Image>("horizon_vio/horizon_vio_track_image", 5));
    // path_publisher_ = std::make_shared<ros::Publisher>(visualizer_nh.advertise<ROS_Path>("horizon_vio/horizon_vio_path", 5));
    odom_publisher_ = std::make_shared<ros::Publisher>(visualizer_nh.advertise<ROS_Odom>("/camera/odom/sample", 5));

    tf_msg_.header.frame_id = "global";
    tf_msg_.child_frame_id = "camera_color_optical_frame";
    image_header_.frame_id = "camera_color_optical_frame";

    draw_thread_ = std::make_shared<std::thread>(std::bind(
            &ROSVisualizer::DrawTrajectory, this));
  }

  void GetConfigFilePath(std::string &config_file_path){
    config_file_path = config_file_path_;
  }

  void PublishPose(const Eigen::Matrix<double, 3, 3>&R_GtoCi,
                   const Eigen::Matrix<double, 3, 1>&p_CioinG,
                   double ts,
                   const std::string &child_id) {
    {
      std::lock_guard<std::mutex> lck(state_mutex_);
      tf_msg_.header.stamp.sec = ts;
      tf_msg_.header.stamp.nsec = (ts - tf_msg_.header.stamp.sec) * 1e9;
      rotationMatrix_ = R_GtoCi;
      translationMatrix_ = p_CioinG;
      is_pose_updated_ = true;
      tf_msg_.child_frame_id = child_id;
    }
  }

  void PublishRenderedImage(cv::Mat &img, double ts, const std::string &frame_id) {
    {
      std::lock_guard<std::mutex> lck(img_mtx_);
      trackHistoryImg_ = img;
      image_header_.stamp.sec = ts;
      image_header_.stamp.nsec = (ts - image_header_.stamp.sec) * 1e9;
      image_header_.frame_id = frame_id;
      is_img_updated_ = true;
    }
  }

  void ShutDown() {
    if (!is_running_) return;
    is_running_ = false;
    draw_thread_->join();
  }

private:
  std::mutex state_mutex_;
  bool is_pose_updated_;
  Eigen::Matrix<double, 3, 3> rotationMatrix_;
  Eigen::Matrix<double, 3, 1> translationMatrix_;

  nav_msgs::Path path_msg_;
  ROS_Odom odom_msg_;
  geometry_msgs::TransformStamped tf_msg_;

  bool is_img_updated_;
  std::mutex img_mtx_;
  std_msgs::Header image_header_;
  cv::Mat trackHistoryImg_, currTrackedFeaturesImg_;

  // std::shared_ptr<ros::Publisher> path_publisher_;
  // std::shared_ptr<ros::Publisher> image_publisher_;
  std::shared_ptr<ros::Publisher> odom_publisher_;

  std::shared_ptr<std::thread> draw_thread_;
  std::atomic_bool is_running_;
  std::string config_file_path_;
private:

  void wrap_img(sensor_msgs::Image &img_msg, const cv::Mat &mat) {
    cv_bridge::CvImage img_bridge;
    if (mat.channels() == 1) {
      img_bridge = cv_bridge::CvImage(image_header_, "mono8", mat);
    } else {
      img_bridge = cv_bridge::CvImage(image_header_, "bgr8", mat);
    }
    img_bridge.toImageMsg(img_msg);
  }

  tf2::Transform TransformFromMat () {
    tf2::Matrix3x3 tf_camera_rotation(
            rotationMatrix_(0, 0), rotationMatrix_(0, 1), rotationMatrix_(0, 2),
            rotationMatrix_(1, 0), rotationMatrix_(1, 1), rotationMatrix_(1, 2),
            rotationMatrix_(2, 0), rotationMatrix_(2, 1), rotationMatrix_(2, 2));
    tf2::Vector3 tf_camera_translation (translationMatrix_[0],
                                        translationMatrix_[1],
                                        translationMatrix_[2]);
//    const tf2::Matrix3x3 trans (0, 1, 0,
//                                        0, 0, -1,
//                                        -1, 0, 0);
//    tf_camera_rotation = tf_camera_rotation.transpose() * trans;
   // tf_camera_translation = tf_camera_translation;
    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
  }

  template<typename T>
  bool CheckWhetherAddPose(const geometry_msgs::Pose &position,
          const T& t) {
    auto x = position.position.x - t.x;
    auto y = position.position.y - t.y;
    auto z = position.position.z - t.z;
    return (x * x + y * y + z * z) > 0.001;
  }

  void DrawTrajectory() {
    tf2::Transform tf_transform;
    geometry_msgs::PoseStamped pose_msg;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    double last_time = 0.0;

    while (is_running_) {
      //  Pub trajectory
      {
        std::lock_guard<std::mutex> lck(state_mutex_);
        tf_transform = TransformFromMat();
        tf_msg_.transform = tf2::toMsg(tf_transform);
        // tf_broadcaster.sendTransform(tf_msg_);
        if(last_time != tf_msg_.header.stamp.sec + tf_msg_.header.stamp.nsec * 1e9){
          tf_broadcaster.sendTransform(tf_msg_);
          odom_msg_.header.stamp = tf_msg_.header.stamp;
          odom_msg_.header.frame_id = "odom_frame";
          odom_msg_.pose.pose.orientation = tf_msg_.transform.rotation;
          odom_msg_.pose.pose.position.x = tf_msg_.transform.translation.x;
          odom_msg_.pose.pose.position.y = tf_msg_.transform.translation.y;
          odom_msg_.pose.pose.position.z = tf_msg_.transform.translation.z;
          odom_publisher_->publish(odom_msg_);
          last_time = tf_msg_.header.stamp.sec + tf_msg_.header.stamp.nsec * 1e9;
        }

        // if (is_pose_updated_) {
          // tf_transform = TransformFromMat();
          // tf_msg_.transform = tf2::toMsg(tf_transform);
          // tf_broadcaster.sendTransform(tf_msg_);
          // path_msg_.header = tf_msg_.header;
          // if (path_msg_.poses.empty() || CheckWhetherAddPose(path_msg_.poses.back().pose,
          //         tf_msg_.transform.translation)) {
          //   pose_msg.header = tf_msg_.header;
          //   pose_msg.pose.orientation = tf_msg_.transform.rotation;
          //   pose_msg.pose.position.x = tf_msg_.transform.translation.x;
          //   pose_msg.pose.position.y = tf_msg_.transform.translation.y;
          //   pose_msg.pose.position.z = tf_msg_.transform.translation.z;
          //   path_msg_.poses.push_back(pose_msg);
          //   path_publisher_->publish(path_msg_);
          // }
          // is_pose_updated_ = false;
        // }
      }

      // //  Pub rendered image
      // {
      //   sensor_msgs::Image img_msg;
      //   std::unique_lock<std::mutex> lck(img_mtx_);
      //   if (is_img_updated_) {
      //     wrap_img(img_msg, trackHistoryImg_);
      //     is_img_updated_ = false;
      //     lck.unlock();
      //     image_publisher_->publish(img_msg);
      //   }
      // }

      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "hobot_vio_node");
  time_t t = time(nullptr);
  struct tm* now = localtime(&t);
  std::stringstream timestream;
  timestream << "trans_quat_camera_" << std::setw(2) << std::setfill('0')
             << now->tm_hour << '_' << std::setw(2) << std::setfill('0')
             << now->tm_min << '_' << std::setw(2) << std::setfill('0')
             << now->tm_sec << ".txt";
  std::ofstream out_txt_file;
  out_txt_file.open(timestream.str(), std::ios::out | std::ios::trunc);
  out_txt_file << std::fixed;
  ROSVisualizer::GetInstance();
  std::shared_ptr<RosSubNode> rosSubNode;
  //  Step 1. Loading config file

  std::string path_config;
  ROSVisualizer::GetInstance()->GetConfigFilePath(path_config);

  rosSubNode = std::make_shared<RosSubNode>();

  std::thread([&]() {
    ros::spin();
    rosSubNode->ShutDown();
  }).detach();
  
  auto vio = std::make_shared<HorizonVIO::HorizonVIOSystem>(path_config);
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  uint64 index_img_end = 0;
  cv::Mat image;
  int ret;
  uint64 image_ts;
  uint queue_size;
  while (ros::ok()) {
    // std::chrono::steady_clock::time_point tp1 = std::chrono::steady_clock::now();
    ret = rosSubNode->GetImage(image_ts, image, queue_size);
    // std::chrono::steady_clock::time_point tp2 = std::chrono::steady_clock::now();
    if (ret == 0) {
      std::deque<std::pair<uint64, std::shared_ptr<float>>> imu_datas;
      // std::chrono::steady_clock::time_point tp3 = std::chrono::steady_clock::now();
      ret = rosSubNode->GetImusByTimeStamp(image_ts, imu_datas);
      // std::chrono::steady_clock::time_point tp4 = std::chrono::steady_clock::now();
      if (ret == 0) {
        for (const auto &imu_data : imu_datas) {
          auto imu_ptr = imu_data.second.get();
          double imu_ts = (double) imu_data.first / 1e9;
          HorizonVIO::IMU_MSG imuMsg{};
          imuMsg.timestamp = imu_ts;
          imuMsg.linear_acceleration[0] = imu_ptr[0];
          imuMsg.linear_acceleration[1] = imu_ptr[1];
          imuMsg.linear_acceleration[2] = imu_ptr[2];
          imuMsg.angle_velocity[0] = imu_ptr[3];
          imuMsg.angle_velocity[1] = imu_ptr[4];
          imuMsg.angle_velocity[2] = imu_ptr[5];
          vio->ReceiveImu(imuMsg);
        }
      }
      HorizonVIO::IMG_MSG img_msg{};
      // std::chrono::steady_clock::time_point tp5 = std::chrono::steady_clock::now();
      img_msg.timestamp = (double) image_ts / 1e9;
      img_msg.image = image.clone();
      vio->ReceiveCamera(img_msg);
      // std::chrono::steady_clock::time_point tp6 = std::chrono::steady_clock::now();
      ++index_img_end;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    HorizonVIO::Localization localization{};
    vio->GetLocalization(localization);
    if (localization.status == HorizonVIO::LocalizationStatus::VIO_TRACKING) {
      ROS_WARN("Localization position[x, y, z]: [%f, %f, %f]",
        localization.positizon[0],
        localization.positizon[1],
        localization.positizon[2]);
      ROS_WARN("Image time %.9f" ,localization.timestamp);
      out_txt_file << std::setprecision(9) << localization.timestamp << " ";
      out_txt_file << std::setprecision(6)
                   << localization.positizon[0] << " "
                   << localization.positizon[1] << " "
                   << localization.positizon[2] << " "
                   << localization.quat[0] << " "
                   << localization.quat[1] << " "
                   << localization.quat[2] << " "
                   << localization.quat[3]
                   << std::endl;
    }
    Eigen::Quaterniond Q(localization.quat[3],
                         localization.quat[0],
                         localization.quat[1],
                         localization.quat[2]);
    Eigen::Matrix<double, 3, 1> T(localization.positizon[0],
                                  localization.positizon[1],
                                  localization.positizon[2]);
    ROSVisualizer::GetInstance()->PublishPose(
            Q.toRotationMatrix(), T, localization.timestamp,
            rosSubNode->GetImageFrame());
    // std::chrono::steady_clock::time_point tp8 = std::chrono::steady_clock::now();

  //  std::cout << "2-1: " << std::chrono::duration_cast<
  //          std::chrono::milliseconds>(tp2 - tp1).count() << std::endl;
  //  std::cout << "3-2: " << std::chrono::duration_cast<
  //          std::chrono::milliseconds>(tp3 - tp2).count() << std::endl;
  //  std::cout << "4-3: " << std::chrono::duration_cast<
  //          std::chrono::milliseconds>(tp4 - tp3).count() << std::endl;
  //  std::cout << "5-4: " << std::chrono::duration_cast<
  //          std::chrono::milliseconds>(tp5 - tp4).count() << std::endl;
  //  std::cout << "6-5: " << std::chrono::duration_cast<
  //          std::chrono::milliseconds>(tp6 - tp5).count() << std::endl;
  }
  out_txt_file.close();
  ros::shutdown();
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  ROS_WARN("Time for the total: %f s",std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count());
  ROS_WARN("avg time : %.6f", std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() / index_img_end);
  ROSVisualizer::GetInstance()->ShutDown();
  return 0;
}
