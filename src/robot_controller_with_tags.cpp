#include "rclcpp/rclcpp.hpp"
// #include "planning/david_planning_three.hh"
#include <geometry_msgs/msg/twist.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode() : Node("robot_controller") {
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/Differential_drive_bot/cmd_vel", 10);
        
        // Subscriber for AprilTag detections
        apriltag_subscriber_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/tag_detections", 10, std::bind(&RobotControllerNode::tagCallback, this, std::placeholders::_1)
        );
        
        // timer to periodically send commands
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotControllerNode::controlLoop, this));
    }

private:
    void tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        if (!msg->detections.empty()) {
            RCLCPP_INFO(this->get_logger(), "Tag detected: ID %d", msg->detections[0].id);
            // will add position data about april tags
        }
    }

    void controlLoop() {
        // sending velocity commands
        auto cmd_msg = geometry_msgs::msg::Twist();
        // Example commands for now
        cmd_msg.linear.x = 0.5;  // move forward
        cmd_msg.angular.z = 0.1; // turn slightly
        velocity_publisher_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControllerNode>());
    rclcpp::shutdown();
    return 0;
}
