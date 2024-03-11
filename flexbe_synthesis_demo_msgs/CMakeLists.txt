cmake_minimum_required(VERSION 3.5)

project(flexbe_synthesis_demo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
    "msg/WGCFStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ADD_LINTER_TESTS
    DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
