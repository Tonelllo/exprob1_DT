#include <algorithm>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include "robotMover.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <string>

using std::placeholders::_1;

/**
 * @brief Constructor for the movement of the robot
 */
RobotMover::RobotMover() : Node("aruco_controller")
{
  mCameraSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 1, std::bind(&RobotMover::getCurrentFrame, this, _1));
  mDetectionPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("/assignment/detected_markers", 1);
  mVelocityPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  mCurrentSearchingIndex_ = 0;
  startRotation();
}

/**
 * @brief Gets the current image for detection of the aruco
 *
 * @param img Current image got by the camera
 */
void RobotMover::getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr img)
{
    mArucoDetector_.detect(img);

  size_t index = 0;
  for (const auto& id : mArucoDetector_.markerIds_)
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
      auto tl = mArucoDetector_.markerCorners_[index][0];
      auto br = mArucoDetector_.markerCorners_[index][2];
      auto xCenter = (tl.x + br.x) / 2;
      auto yCenter = (tl.y + br.y) / 2;
      float radius = cv::norm(tl - br) / 2;

      if (xCenter >= (float)mArucoDetector_.currentFrame_.cols / 2 - 20 && xCenter <= (float)mArucoDetector_.currentFrame_.cols / 2 + 20)
      {
        if (id == mDetectedIds_[mCurrentSearchingIndex_])
        {
          RCLCPP_INFO(this->get_logger(), "Published image of marker id: %d", id);
          std::string displayString = "Id: " + std::to_string(id);
          cv::circle(mArucoDetector_.currentFrame_, cv::Point(xCenter, yCenter), radius, cv::Scalar(0, 255, 0), 3);
          cv::putText(mArucoDetector_.currentFrame_, displayString, cv::Point(xCenter + radius, yCenter - radius),
                      cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 3);

          // ### Uncomment this section to have directly the visualizations of the detection
          // cv_bridge::CvImagePtr p = cv_bridge::toCvCopy(*mArucoDetector_.getFrameAsImgMsg(), sensor_msgs::image_encodings::BGR8);
          // cv::Mat m = p->image;
          // cv::imshow("MsgPreview", m);
          // cv::waitKey(10);

          mDetectionPublisher_->publish(*mArucoDetector_.getFrameAsImgMsg());
          ++mCurrentSearchingIndex_;
          if (mCurrentSearchingIndex_ == 5)
            mCurrentSearchingIndex_ = 0;
        }
      }
    }
    ++index;
  }
}

/**
 * @brief Starts the rotation of the robot
 *
 * Note that on the real robot this has to be called with a timer
 */
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

/**
 * @brief Stops the rotation of the robot
 */
void RobotMover::stopRotation()
{
  RCLCPP_INFO(this->get_logger(), "Stopped rotation");
  geometry_msgs::msg::Twist cmdVel;
  cmdVel.angular.z = 0;
  mVelocityPublisher_->publish(cmdVel);
}
