#include <rclcpp/rclcpp.hpp>

class MovementController : public rclcpp::Node {
    rclcpp::TimerBase::SharedPtr mtimer_;
public:
    MovementController();
    void timerCallback();
};
