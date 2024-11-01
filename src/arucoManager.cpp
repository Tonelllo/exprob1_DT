#include "arucoManager.hpp"
#include <algorithm>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

ArucoManager::ArucoManager(std::vector<int>& detectedIds) : Node("aruco_controller"), mDetectedIds_(detectedIds)
{
  mArucoSubscriber_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "/aruco_markers", 1, std::bind(&ArucoManager::arucoCallback, this, _1));
  mCameraSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 1, std::bind(&ArucoManager::getCurrentFrame, this, _1));
  RCLCPP_INFO(this->get_logger(), "ArucoManager created");
}

void ArucoManager::arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  size_t index = 0;
  for (const auto& id : msg->marker_ids)
  {
    auto itr = std::find(mDetectedIds_.begin(), mDetectedIds_.end(), id);
    if ((mDetectedIds_.size() == 0 || itr == mDetectedIds_.end()))
    {
      if (mDetectedIds_.size() < 5)
      {
        mDetectedIds_.emplace_back(id);
        if (mDetectedIds_.size() == 5)
        {
          std::sort(mDetectedIds_.begin(), mDetectedIds_.end());
          for (const auto& elem : mDetectedIds_)
          {
            std::cout << elem << std::endl;
          }
        }
      }
    }
    else if (mDetectedIds_.size() == 5)
    {
      auto xPos = msg->poses[index].position.x;
      if (xPos >= -0.05 && xPos <= 0.05)
      {
        if (id == mDetectedIds_[mCurrentSearchingIndex_])
        {
          RCLCPP_INFO(this->get_logger(), "SHOWING MARKER");
          mFrameMutex_.lock();
          cv::circle(mCurrentFrame_, cv::Point(0, 0), 100, cv::Scalar(0, 255, 0), 3);
          cv::imshow("test", mCurrentFrame_);
          mFrameMutex_.unlock();
          cv::waitKey(100);
          ++mCurrentSearchingIndex_;
        }
      }
    }
  }
  ++index;
}

void ArucoManager::getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr img)
{
  cv_bridge::CvImagePtr cvPtr;
  cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  // TODO copying every time
  mFrameMutex_.lock();
  mCurrentFrame_ = cvPtr->image;
  mFrameMutex_.unlock();
}
