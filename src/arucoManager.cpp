#include "arucoManager.hpp"

using std::placeholders::_1;

ArucoManager::ArucoManager(std::vector<int>& detectedIds) : Node("aruco_controller"), mDetectedIds_(detectedIds)
{
  mArucoSubscriber_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
      "/aruco_markers", 1, std::bind(&ArucoManager::arucoCallback, this, _1));
  RCLCPP_INFO(this->get_logger(), "ArucoManager created");
}

void ArucoManager::arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
  for (const auto& id : msg->marker_ids)
  {
    auto itr = std::find(mDetectedIds_.begin(), mDetectedIds_.end(), id);
    if (mDetectedIds_.size() == 0 || itr == mDetectedIds_.end())
    {
      mDetectedIds_.emplace_back(id);
    }
  }
}
