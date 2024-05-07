cmake_minimum_required(VERSION 3.0.2)
project(tortoisebotpromax_odometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs nav_msgs rospy tf2_ros
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs rospy tf2_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
install(FILES config/ekf.yaml config/icp.yaml config/lidar.lua
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(FILES launch/carto_odometry.launch launch/icp_fuse.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/alpha_beta_filter.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

