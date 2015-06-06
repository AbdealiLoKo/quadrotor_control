cmake_minimum_required(VERSION 2.8.3)
project(rl_experiment)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rl_msgs tf rl_common rl_agent rl_env)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rl_experiment
  CATKIN_DEPENDS roscpp std_msgs rl_msgs tf rl_common rl_agent rl_env
  DEPENDS
)

add_executable(experiment
  src/rl.cc
)

target_link_libraries(experiment ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch params DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
