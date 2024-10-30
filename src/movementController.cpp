#include "movementController.hpp"
#include <rclcpp/logging.hpp>

MovementController::MovementController() : Node("movement_controller"){
    using namespace std::chrono_literals;
    mVelocityPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void MovementController::startRotation(){
    RCLCPP_INFO(this->get_logger(), "Started rotation");
    geometry_msgs::msg::Twist cmdVel;
    cmdVel.angular.z = 1;
    mVelocityPublisher_->publish(cmdVel);
}

void MovementController::stopRotation(){
    RCLCPP_INFO(this->get_logger(), "Stopped rotation");
    geometry_msgs::msg::Twist cmdVel;
    cmdVel.angular.z = 0;
    mVelocityPublisher_->publish(cmdVel);
}
