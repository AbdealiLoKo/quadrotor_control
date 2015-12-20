cmake_minimum_required(VERSION 2.8.3)
project(rl_common)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp
             std_msgs message_generation geometry_msgs)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

add_message_files(
  FILES
  RLAction.msg
  RLExperimentInfo.msg
  RLStateReward.msg
)

add_service_files(
  FILES
  RLRunSim.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

add_library(rlcommon
  src/core.cc
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rlcommon
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
