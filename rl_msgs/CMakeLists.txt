cmake_minimum_required(VERSION 2.8.3)
project(rl_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

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
#  INCLUDE_DIRS include
#  LIBRARIES rl_msgs
  CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS system_lib
)
