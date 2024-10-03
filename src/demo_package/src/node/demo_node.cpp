#include "rclcpp/rclcpp.hpp"

class DemoNode : public rclcpp::Node {
public:
    DemoNode() : Node("demo_node_instance") {
        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            RCLCPP_INFO(get_logger(), "Tick #%ld", ticksCount_);
            ticksCount_++;
        });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    size_t ticksCount_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<DemoNode>());
    RCLCPP_WARN(rclcpp::get_logger("some_logger"), "rclcpp::shutdown()");
    rclcpp::shutdown();
    return 0;
}
