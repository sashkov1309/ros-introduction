#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("node_subscriber");

    auto callback = [&](const std_msgs::msg::String& message){
        RCLCPP_INFO_STREAM(node->get_logger(), "Received [" << message.data << "]");
    };
    auto subscriber = node->create_subscription<std_msgs::msg::String>("/demo/pubsub", 10, callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
