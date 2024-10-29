#include "movementController.hpp"

MovementController::MovementController() : Node("movement_controller"){
    using namespace std::chrono_literals;
    this->create_wall_timer(500ms, std::bind(&MovementController::timerCallback, this));
}

void MovementController::timerCallback(){
    printf("hello\n");
    /*RCLCPP_INFO(this->get_logger(), "running");*/
}
