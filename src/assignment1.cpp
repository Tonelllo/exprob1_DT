#include<movementController.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  printf("hello world exprob package\n");
  /*MovementController mc;*/

  rclcpp::spin(std::make_shared<MovementController>());
  rclcpp::shutdown();
  return 0;
}
