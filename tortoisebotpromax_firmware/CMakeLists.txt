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
# install(PROGRAMS
#   scripts/rbl_logger.py scripts/robot_diagnostics.py
#   DESTINATION lib/${PROJECT_NAME}
# )
install(PROGRAMS
  compiled_scripts/rbl_logger.pyc compiled_scripts/robot_diagnostics.pyc
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  bin/goal_status_publisher
  bin/network_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
# install(
#   DIRECTORY bin/
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
