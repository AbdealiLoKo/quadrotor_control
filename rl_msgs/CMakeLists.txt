cmake_minimum_required(VERSION 2.8.3)
project(rl_msgs)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geometry_msgs)

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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rl_msgs
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)
