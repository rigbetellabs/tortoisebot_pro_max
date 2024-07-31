cmake_minimum_required(VERSION 3.8)
project(tortoisebotpromax_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/rbl_logger.py scripts/robot_diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(goal_status_publisher src/goal_status_publisher.cpp)
add_executable(network_status_publisher src/network_status_publisher.cpp)
# Add dependencies to the executable
ament_target_dependencies(goal_status_publisher rclcpp std_msgs action_msgs)
ament_target_dependencies(network_status_publisher rclcpp std_msgs action_msgs)
install(
  DIRECTORY launch scripts config src
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
goal_status_publisher
network_status_publisher
  DESTINATION lib/${PROJECT_NAME})
#Required for dependencies and other folder
# install(
#   DIRECTORY 
#   scripts
#   DESTINATION share/${PROJECT_NAME}
# )
ament_package()
