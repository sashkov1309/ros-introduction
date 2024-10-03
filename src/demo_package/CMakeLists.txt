cmake_minimum_required(VERSION 3.8)
project(demo_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(demo_dto REQUIRED)

# Node #################################################################################################################

add_executable(demo_node src/node/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp)

# PubSub ###############################################################################################################

add_executable(demo_publisher src/pubsub/demo_publisher.cpp)
ament_target_dependencies(demo_publisher rclcpp std_msgs demo_dto)

add_executable(demo_subscriber src/pubsub/demo_subscriber.cpp)
ament_target_dependencies(demo_subscriber rclcpp std_msgs demo_dto)

# Service-Client #######################################################################################################

add_executable(demo_server src/service/demo_service.cpp)
ament_target_dependencies(demo_server rclcpp std_msgs demo_dto)

add_executable(demo_client src/service/demo_client.cpp)
ament_target_dependencies(demo_client rclcpp std_msgs demo_dto)

########################################################################################################################

install(TARGETS
        demo_node
        demo_publisher
        demo_subscriber
        demo_server
        demo_client
        DESTINATION
        lib/${PROJECT_NAME}
)

ament_package()
