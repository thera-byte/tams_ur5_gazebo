cmake_minimum_required(VERSION 2.8.3)
project(tams_ur5_gazebo)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES grasp.action
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_gazebo
#  CATKIN_DEPENDS ur_description
#  DEPENDS system_lib
  CATKIN_DEPENDS actionlib_msgs
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
