#include "rclcpp/rclcpp.hpp"

#include "demo_dto/srv/addition.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("demo_node_client");

    rclcpp::Client<demo_dto::srv::Addition>::SharedPtr client_ = node->create_client<demo_dto::srv::Addition>("/demo/service");

    auto request_message = std::make_shared<demo_dto::srv::Addition::Request>();
    srand(time(nullptr));
    request_message->a = rand() % 10;
    request_message->b = rand() % 10;

    RCLCPP_INFO(node->get_logger(), "Asking to add [%lf] + [%lf]", request_message->a, request_message->b);

    if (not client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node->get_logger(), "Failed to find server");
    }

    auto response = client_->async_send_request(request_message);
    rclcpp::spin_until_future_complete(node, response);

    RCLCPP_INFO(node->get_logger(), "Received result [%lf]", response.get()->c);

    rclcpp::shutdown();
    return 0;
}
