#include <movementController.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  MovementController movController;

  rclcpp::sleep_for(3s);
  movController.startRotation();

  rclcpp::sleep_for(3s);
  movController.stopRotation();
  rclcpp::shutdown();
  return 0;
}
