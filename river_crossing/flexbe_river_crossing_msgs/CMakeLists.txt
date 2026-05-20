cmake_minimum_required(VERSION 3.5)

project(flexbe_river_crossing_msgs)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
