#pragma once
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

class ArucoManager : public rclcpp::Node
{
  void getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr);

  std::vector<int>& mDetectedIds_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCameraSubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDetectionPublisher_;
  size_t mCurrentSearchingIndex_;
  std::vector<std::vector<cv::Point2f>> mMarkerCorners_;
  cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams_;
  cv::Ptr<cv::aruco::Dictionary> mDict_;
public:
  ArucoManager(std::vector<int>&);
};
