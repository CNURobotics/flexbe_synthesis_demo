cmake_minimum_required(VERSION 3.10)
project(flexbe_river_crossing_behaviors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY
  manifest
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_copyright "test/test_copyright.py")
  ament_add_pytest_test(test_flake8 "test/test_flake8.py")
  ament_add_pytest_test(test_pep257 "test/test_pep257.py")
endif()

ament_package()
