#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <vector>

class RobotMover : public rclcpp::Node
{
  void getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr);
  void rotatingRobot(cv::Mat&);

  cv_bridge::CvImagePtr mCvPtr_;
  cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams_;
  cv::Ptr<cv::aruco::Dictionary> mDict_;
  size_t mCurrentSearchingIndex_;
  std::vector<int>& mDetectedIds_;
  std::vector<int> mMarkerIds_;
  std::vector<std::vector<cv::Point2f>> mMarkerCorners_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelocityPublisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDetectionPublisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCameraSubscriber_;

public:
  RobotMover(std::vector<int>&);
  void startRotation();
  void stopRotation();
};
