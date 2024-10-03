#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("node_for_publisher");

    auto publisher = node->create_publisher<std_msgs::msg::String>("/demo/pubsub", 10);

    long ticks = 0;
    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(std::chrono::seconds(1), [&]{
        RCLCPP_INFO(node->get_logger(), "Publishing ticks [%ld]", ticks);

        std_msgs::msg::String message;
        message.data = std::to_string(ticks);
        publisher->publish(message);

        ++ticks;
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
