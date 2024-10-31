#pragma once
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>

class ArucoManager : public rclcpp::Node
{
  std::vector<int>& mDetectedIds_;
  void arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr);
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr mArucoSubscriber_;

public:
  ArucoManager(std::vector<int>&);
};
