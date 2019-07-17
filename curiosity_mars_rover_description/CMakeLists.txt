cmake_minimum_required(VERSION 2.8.3)
project(curiosity_mars_rover_description)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  genmsg
  actionlib_msgs
  gazebo_ros
)

add_action_files(
  DIRECTORY action
  FILES CuriosityArm.action
  FILES CuriosityMast.action
  FILES CuriosityWheels.action
)

find_package(gazebo REQUIRED)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Depend on system install of Gazebo


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    actionlib_msgs
)

catkin_install_python(PROGRAMS
  scripts/arm_client.py
  scripts/arm_server.py
  scripts/mast_client.py
  scripts/mast_server.py
  scripts/wheels_client.py
  scripts/wheels_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
