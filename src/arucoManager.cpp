#include "arucoManager.hpp"
#include <algorithm>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

ArucoManager::ArucoManager(std::vector<int>& detectedIds) : Node("aruco_controller"), mDetectedIds_(detectedIds)
{
  mCameraSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 1, std::bind(&ArucoManager::getCurrentFrame, this, _1));
  mDetectorParams_ = cv::aruco::DetectorParameters::create();
  mDict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  mDetectionPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("/assignment/detected_markers", 1);
}

void ArucoManager::getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr img)
{
  cv_bridge::CvImagePtr cvPtr;
  cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  cv::Mat currentFrame = cvPtr->image;
  std::vector<int> markerIds;
  cv::aruco::detectMarkers(currentFrame, mDict_, mMarkerCorners_, markerIds, mDetectorParams_);
  /*cv::Mat outputImage = currentFrame.clone();*/
  /*cv::aruco::drawDetectedMarkers(outputImage, mMarkerCorners_, markerIds);*/
  /*cv::imshow("detectedArucos", outputImage);*/
  /*cv::waitKey(100);*/

  size_t index = 0;
  for (const auto& id : markerIds)
  {
    auto itr = std::find(mDetectedIds_.begin(), mDetectedIds_.end(), id);
    if ((mDetectedIds_.size() == 0 || itr == mDetectedIds_.end()))
    {
      if (mDetectedIds_.size() < 5)
      {
        RCLCPP_INFO(this->get_logger(), "Detected new aruco with id: %d", id);
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
      // NOTE that the order is from top left clockwise
      // NOTE that the the corners are given with the ORIGINAL order wich means with the correct orientation
      auto tl = mMarkerCorners_[index][0];
      /*auto tr = mMarkerCorners_[index][1];*/
      auto br = mMarkerCorners_[index][2];
      /*auto bl = mMarkerCorners_[index][3];*/
      auto xCenter = (tl.x + br.x) / 2;
      auto yCenter = (tl.y + br.y) / 2;
      float radius = cv::norm(tl - br) / 2;
      /*RCLCPP_INFO(this->get_logger(), "tl.x: %f, tr.x: %f, bl.x: %f, br.x: %f", tl.x, tr.x, bl.x, br.x);*/

      if (xCenter >= (float)img->width / 2 - 10 && xCenter <= (float)img->width / 2 + 10)
      {
        if (id == mDetectedIds_[mCurrentSearchingIndex_])
        {
          /*RCLCPP_INFO(this->get_logger(), "SHOWING MARKER");*/
          std::string displayString = "Id: " + std::to_string(id);
          cv::circle(currentFrame, cv::Point(xCenter, yCenter), radius, cv::Scalar(0, 255, 0), 3);
          cv::putText(currentFrame, displayString, cv::Point(xCenter + radius, yCenter - radius),
                      cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 3);
          /*cv::imshow("test", currentFrame);*/
          /*cv::waitKey(100);*/

          mDetectionPublisher_->publish(*cvPtr->toImageMsg());
          ++mCurrentSearchingIndex_;
          if (mCurrentSearchingIndex_ == 5)
            mCurrentSearchingIndex_ = 0;
        }
      }
    }
    ++index;
  }
}
