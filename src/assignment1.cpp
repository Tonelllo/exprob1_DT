#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include "camMover.hpp"
#include "robotMover.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

enum class States
{
  SCANNING_FOR_MINIMUM
};

int main(int argc, char** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);


  std::cout << "STARTED" << std::endl;
  /*rclcpp::spin(std::make_shared<CamMover>());*/
  rclcpp::spin(std::make_shared<RobotMover>());

  rclcpp::shutdown();
  return 0;
}
