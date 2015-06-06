cmake_minimum_required(VERSION 2.8.3)
project(rl_py)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs tf message_generation)
include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
  RLAction.msg
  RLEnvDescription.msg
  RLEnvSeedExperience.msg
  RLExperimentInfo.msg
  RLStateReward.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs tf message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
