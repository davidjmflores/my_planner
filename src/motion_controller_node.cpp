#include "rclcpp/rclcpp.hpp"
#include "planning/david_planning_three.hh"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>


//ros2 topic pub /target_coordintes geometry_msgs/msg/Point "{x: 2.0, y: 3.0, z: 0.0}"
class MotionControllerNode : public rclcpp::Node {
public:
    MotionControllerNode() : Node("motion_controller") {
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/Differential_drive_bot/cmd_vel", 10);
        // Subscriber for target coordinate commands
        target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_coordinates", 10, std::bind(&MotionControllerNode::targetCallback, this, std::placeholders::_1));
        // timer to periodically send commands
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotionControllerNode::controlLoop, this));
    }

private:
    void targetCallback(const geometry_msgs::msg::Point::SharedPtr target)
    {
        target_x_ = target->x;
        target_y_ = target->y;
        has_target_ = true;
    }
    void controlLoop() {
        if(!has_target_) return;

        geometry_msgs::msg::Twist velocity_msg;
        double error_x = target_x_ - current_x_;
        double error_y = target_y_ - current_y_;

        double kp = 0.5;
        velocity_msg.linear.x = kp * error_x;
        velocity_msg.linear.y = kp * error_y;

        RCLCPP_INFO(this->get_logger(), "Sending velocity: x=%.2f, y=%.2f", velocity_msg.linear.x, velocity_msg.linear.y);
        velocity_publisher_->publish(velocity_msg);

        current_x_ += velocity_msg.linear.x * 0.1;
        current_y_ += velocity_msg.linear.y * 0.1;

        if(std::hypot(error_x, error_y) < 0.1) {
            RCLCPP_INFO(this->get_logger(),"Reached Target.");
            has_target_ = false;
            velocity_msg.linear.x = 0.0;
            velocity_msg.linear.y = 0.0;
            velocity_publisher_->publish(velocity_msg);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double target_x_ = 0.0, target_y_ = 0.0;
    double current_x_ = 0.0, current_y_ = 0.0;
    bool has_target_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionControllerNode>());
    rclcpp::shutdown();
    return 0;
}
