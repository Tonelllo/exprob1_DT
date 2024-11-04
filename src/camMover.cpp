#include "camMover.hpp"
#include <cstddef>
#include <iterator>
#include <memory>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

CamMover::CamMover() : Node("camMover")
{
  mDetectionPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("/assignment/detected_markers", 1);
  mCurrentSearchingIndex_ = 0;
  mImageSubscriber_.subscribe(this, "/camera/image_raw");
  mJointSubscriber_.subscribe(this, "/joint_states");
  mSyncronizer_ = std::make_shared<mSync_>(mSyncPolicy_(10), mImageSubscriber_, mJointSubscriber_);
  mSyncronizer_->setAgePenalty(0.5);
  mSyncronizer_->registerCallback(std::bind(&CamMover::getCurrentFrame, this, _1, _2));
  mDetectorParams_ = cv::aruco::DetectorParameters::create();
  mDict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

  mVelocityPublisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/camera_velocity_controller/commands", 10);
  startRotation();
}
void CamMover::getCurrentFrame(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                               const sensor_msgs::msg::JointState::ConstSharedPtr& js)
{
  try
  {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat currentFrame = mCvPtr_->image;
  cv::aruco::detectMarkers(currentFrame, mDict_, mMarkerCorners_, mMarkerIds_, mDetectorParams_);
  cv::aruco::drawDetectedMarkers(currentFrame, mMarkerCorners_, mMarkerIds_);
  cv::imshow("test", currentFrame);
  cv::waitKey(10);
  mCurrentJointPos_ = js->position[0];
  size_t index = 0;
  for (const auto& id : mMarkerIds_)
  {
    auto tl = mMarkerCorners_[index][0];
    /*auto tr = mMarkerCorners_[index][1];*/
    auto br = mMarkerCorners_[index][2];
    /*auto bl = mMarkerCorners_[index][3];*/
    auto xCenter = (tl.x + br.x) / 2;
    auto itr = std::find_if(mDetectedIds_.begin(), mDetectedIds_.end(), [&id](auto elem) {
      if (elem.first == id)
      {
        return true;
      }
      return false;
    });
    if (mDetectedIds_.size() < 5 && itr == mDetectedIds_.end())
    {
      if (xCenter < (float)currentFrame.cols / 2 - 10 || xCenter > (float)currentFrame.cols / 2 + 10)
        continue;
      std::cout << "L: " << (float)currentFrame.cols / 2 - 5 << "\tC: " << xCenter
                << "\tR: " << (float)currentFrame.cols / 2 + 5 << std::endl;
      std::cout << "Detected new aruco with id: " << id << std::endl;
      // 0 is the index of camera_joint in the array
      mDetectedIds_.insert({ id, mCurrentJointPos_ });
      if (mDetectedIds_.size() == 5)
      {
        /*std::sort(mDetectedIds_.begin(), mDetectedIds_.end());*/
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
          outString += std::to_string(elem.first);
        }
        RCLCPP_INFO(this->get_logger(), "Found all markers. The order is: %s", outString.c_str());
        stopRotation();
        mTarget_ = mDetectedIds_.begin()->second;
        mTimer_ = this->create_wall_timer(25ms, std::bind(&CamMover::timerCallback, this));
      }
      std::cout << "DIMENSION: " << mDetectedIds_.size() << std::endl;
    }
    else if (mDetectedIds_.size() == 5)
    {
      // NOTE that the order is from top left clockwise
      // NOTE that the the corners are given with the ORIGINAL order wich means with the correct orientation
      auto tl = mMarkerCorners_[index][0];
      auto br = mMarkerCorners_[index][2];
      auto xCenter = (tl.x + br.x) / 2;
      auto yCenter = (tl.y + br.y) / 2;
      float radius = cv::norm(tl - br) / 2;
      std::cout << "L: " << (float)currentFrame.cols / 2 - 20 << "\tC: " << xCenter
                << "\tR: " << (float)currentFrame.cols / 2 + 20 << std::endl;

      if (xCenter >= (float)currentFrame.cols / 2 - 60 && xCenter <= (float)currentFrame.cols / 2 + 60)
      {
        // TODO think of a better way
        auto iter = mDetectedIds_.begin();
        std::advance(iter, mCurrentSearchingIndex_);
        if (id == iter->first)
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
          auto auxIter = mDetectedIds_.begin();
          std::advance(auxIter, mCurrentSearchingIndex_);
          mTarget_ = auxIter->second;
        }
      }
    }
    ++index;
  }
}
void CamMover::startRotation()
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(this->get_logger(), "Started rotation");
  std_msgs::msg::Float64MultiArray cmdVel;
  cmdVel.data.emplace_back(0.5);
  while (!mVelocityPublisher_->get_subscription_count())
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for robot to come up");
    rclcpp::sleep_for(1s);
  }
  mVelocityPublisher_->publish(cmdVel);
}

void CamMover::stopRotation()
{
  RCLCPP_INFO(this->get_logger(), "Stopped rotation");
  std_msgs::msg::Float64MultiArray cmdVel;
  cmdVel.data.emplace_back(0);
  mVelocityPublisher_->publish(cmdVel);
}

float CamMover::mod(float num, float base)
{
  /*return num - floor(num / base) * base;*/
  return std::fmod((std::fmod(num, base) + base), base);
}

void CamMover::timerCallback()
{
  float diff = mTarget_ - mod(mCurrentJointPos_, 2 * M_PI);

  diff = mod(diff + M_PI, 2 * M_PI) - M_PI;

  diff = fmin(1, diff);
  diff = fmax(-1, diff);
  std_msgs::msg::Float64MultiArray cmdVel;
  cmdVel.data.emplace_back(0.95 * diff);
  mVelocityPublisher_->publish(cmdVel);
}
