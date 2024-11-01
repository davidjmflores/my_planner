#include "rclcpp/rclcpp.hpp"
#include "planning/david_planning_three.hh"
#include <geometry_msgs/msg/twist.hpp>

class MotionControllerNode : public rclcpp::Node {
public:
    MotionControllerNode() : Node("motion_controller") {
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/Differential_drive_bot/cmd_vel", 10);
        
        // timer to periodically send commands
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotionControllerNode::controlLoop, this));
    }

private:
    void controlLoop() {
        // sending velocity commands
        auto cmd_msg = geometry_msgs::msg::Twist();
        // Example commands for now
        cmd_msg.linear.x = 0.5;  // move forward
        cmd_msg.angular.z = 0.1; // turn slightly
        velocity_publisher_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControllerNode>());
    rclcpp::shutdown();
    return 0;
}
