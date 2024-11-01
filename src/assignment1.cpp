#include "arucoManager.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <memory>
#include <movementController.hpp>
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

  MovementController movController;
  States state = States::SCANNING_FOR_MINIMUM;
  std::vector<int> detectedIds;

  movController.startRotation();

  std::thread t([&detectedIds]() { rclcpp::spin(std::make_shared<ArucoManager>(detectedIds)); });
  std::cout << "STARTED" << std::endl;

  while (true)
  {
    switch (state)
    {
      case States::SCANNING_FOR_MINIMUM:
        break;
    }
    rclcpp::sleep_for(1s);
  }

  /*movController.stopRotation();*/
  t.join();
  rclcpp::shutdown();
  return 0;
}
