#pragma once
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MovementController : public rclcpp::Node
{
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelocityPublisher_;

public:
  MovementController();
  void startRotation();
  void stopRotation();
};
