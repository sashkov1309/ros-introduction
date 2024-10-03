#include "rclcpp/rclcpp.hpp"

#include "demo_dto/srv/addition.hpp"

void AddTwoDoubles(const demo_dto::srv::Addition::Request::SharedPtr request, demo_dto::srv::Addition::Response::SharedPtr response) {
    response->c = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("service_logger"), "[%lf] + [%lf] = [%lf]", request->a, request->b, response->c);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_server");

    auto service = node->create_service<demo_dto::srv::Addition>("/demo/service", &AddTwoDoubles);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
