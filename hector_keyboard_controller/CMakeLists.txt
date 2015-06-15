cmake_minimum_required(VERSION 2.8.3)
project(hector_keyboard_controller)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch params DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
