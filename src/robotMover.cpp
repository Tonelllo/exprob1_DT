#include "robotMover.hpp"
#include <algorithm>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <string>

using std::placeholders::_1;

RobotMover::RobotMover(std::vector<int>& detectedIds) : Node("aruco_controller"), mDetectedIds_(detectedIds)
{
  mCameraSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 1, std::bind(&RobotMover::getCurrentFrame, this, _1));
  mDetectionPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("/assignment/detected_markers", 1);
  mDetectorParams_ = cv::aruco::DetectorParameters::create();
  mDict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  mVelocityPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void RobotMover::rotatingRobot(cv::Mat& currentFrame)
{
  size_t index = 0;
  for (const auto& id : mMarkerIds_)
  {
    auto itr = std::find(mDetectedIds_.begin(), mDetectedIds_.end(), id);
    if (mDetectedIds_.size() < 5 && itr == mDetectedIds_.end())
    {
      RCLCPP_INFO(this->get_logger(), "Detected new aruco with id: %d", id);
      mDetectedIds_.emplace_back(id);
      if (mDetectedIds_.size() == 5)
      {
        std::sort(mDetectedIds_.begin(), mDetectedIds_.end());
        std::string outString;
        bool first = true;
        for (const auto& elem : mDetectedIds_)
        {
          if (!first)
          {
            outString += "\t";
          }
          else
          {
            first = false;
          }
          outString += std::to_string(elem);
        }
        RCLCPP_INFO(this->get_logger(), "Found all markers. The order is: %s", outString.c_str());
      }
    }
    else if (mDetectedIds_.size() == 5)
    {
      // NOTE that the order is from top left clockwise
      // NOTE that the the corners are given with the ORIGINAL order wich means with the correct orientation
      auto tl = mMarkerCorners_[index][0];
      /*auto tr = mMarkerCorners_[index][1];*/
      auto br = mMarkerCorners_[index][2];
      /*auto bl = mMarkerCorners_[index][3];*/
      auto xCenter = (tl.x + br.x) / 2;
      auto yCenter = (tl.y + br.y) / 2;
      float radius = cv::norm(tl - br) / 2;

      if (xCenter >= (float)currentFrame.cols / 2 - 10 && xCenter <= (float)currentFrame.cols / 2 + 10)
      {
        if (id == mDetectedIds_[mCurrentSearchingIndex_])
        {
          RCLCPP_INFO(this->get_logger(), "Published image of marker id: %d", id);
          std::string displayString = "Id: " + std::to_string(id);
          cv::circle(currentFrame, cv::Point(xCenter, yCenter), radius, cv::Scalar(0, 255, 0), 3);
          cv::putText(currentFrame, displayString, cv::Point(xCenter + radius, yCenter - radius),
                      cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 3);

          mDetectionPublisher_->publish(*mCvPtr_->toImageMsg());
          ++mCurrentSearchingIndex_;
          if (mCurrentSearchingIndex_ == 5)
            mCurrentSearchingIndex_ = 0;
        }
      }
    }
    ++index;
  }
}

void RobotMover::getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr img)
{
  mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  cv::Mat currentFrame = mCvPtr_->image;
  cv::aruco::detectMarkers(currentFrame, mDict_, mMarkerCorners_, mMarkerIds_, mDetectorParams_);

  rotatingRobot(currentFrame);
}

void RobotMover::startRotation()
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(this->get_logger(), "Started rotation");
  geometry_msgs::msg::Twist cmdVel;
  cmdVel.angular.z = 1;
  while (!mVelocityPublisher_->get_subscription_count())
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for robot to come up");
    rclcpp::sleep_for(1s);
  }
  mVelocityPublisher_->publish(cmdVel);
}

void RobotMover::stopRotation()
{
  RCLCPP_INFO(this->get_logger(), "Stopped rotation");
  geometry_msgs::msg::Twist cmdVel;
  cmdVel.angular.z = 0;
  mVelocityPublisher_->publish(cmdVel);
}
