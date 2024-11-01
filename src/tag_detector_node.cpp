#include "rclcpp/rclcpp.hpp"
#include "planning/david_planning_three.hh"
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

class TagDetectorNode : public rclcpp::Node {
public:
    TagDetectorNode() : Node("tag_detector") {
        // Subscriber for AprilTag detections
        apriltag_subscriber_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/tag_detections", 10, std::bind(&TagDetectorNode::tagCallback, this, std::placeholders::_1)
        );
    }

private:
    void tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        if (!msg->detections.empty()) {
            RCLCPP_INFO(this->get_logger(), "Tag detected: ID %d", msg->detections[0].id);
            // will add position data about april tags
        }
    }
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}