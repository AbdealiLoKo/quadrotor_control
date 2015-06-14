cmake_minimum_required(VERSION 2.8.3)
project(rl_experiment)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rl_msgs tf rl_common rl_agent rl_env)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rl_experiment
  CATKIN_DEPENDS roscpp std_msgs rl_msgs tf rl_common rl_agent rl_env
  DEPENDS
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch params DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
