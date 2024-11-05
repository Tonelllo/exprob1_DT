#pragma once

#include "arucoDetector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include "Mover.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <vector>

class RobotMover : public rclcpp::Node, public Mover
{
  void getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr);

  ArucoDetector mArucoDetector_;
  size_t mCurrentSearchingIndex_;
  std::vector<int> mDetectedIds_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelocityPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDetectionPublisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCameraSubscriber_;

public:
  RobotMover();
  void startRotation() override;
  void stopRotation() override;
};
