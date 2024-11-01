#pragma once
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <vector>
#include <opencv2/core.hpp>

class ArucoManager : public rclcpp::Node
{
  void arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr);
  void getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr);

  std::vector<int>& mDetectedIds_;
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr mArucoSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCameraSubscriber_;
  size_t mCurrentSearchingIndex_;
  std::mutex mFrameMutex_;
  cv::Mat mCurrentFrame_;

public:
  ArucoManager(std::vector<int>&);
};
