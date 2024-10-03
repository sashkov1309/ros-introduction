cmake_minimum_required(VERSION 3.8)
project(demo_dto)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(demo_dto
        "msg/ArmState.msg"
        "srv/Addition.srv"
        DEPENDENCIES builtin_interfaces
)

ament_package()
