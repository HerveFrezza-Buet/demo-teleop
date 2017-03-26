cmake_minimum_required(VERSION 2.8.3)
project(demo_teleop)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/SafeDroneTeleop.cfg
)


catkin_package()

install(PROGRAMS scripts/key_teleop.py scripts/safe_drone_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
