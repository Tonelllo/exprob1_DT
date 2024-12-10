#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include "camMover.hpp"
#include "robotMover.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  char c;

  std::cout << "STARTED" << std::endl;
  std::cout << "+------------------------------------------------------------+\n"
               "|                                                            |\n"
               "|   Assignment 1 for Experimental robotics laboratory        |\n"
               "|                                                            |\n"
               "|   Press:                                                   |\n"
               "|     > c in order to detect the markers with the camera     |\n"
               "|     > r in order to detect the markers with the movement   |\n"
               "|       of the whole robot                                   |\n"
               "|                                                            |\n"
               "+------------------------------------------------------------+\n"
            << std::endl;

  std::cout << "Selection :";
  std::cin >> c;

  // This is done to stick to the requirement that we should implement the aruco seeking behaviour with two different
  // nodes
  if (c == 'c')
  {
    rclcpp::spin(std::make_shared<CamMover>());
  }
  else
  {
    rclcpp::spin(std::make_shared<RobotMover>());
  }

  rclcpp::shutdown();
  return 0;
}
