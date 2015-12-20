cmake_minimum_required(VERSION 2.8.3)
project(rl_agent)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf rl_common)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs tf rl_common
  DEPENDS gazebo Eigen
)

add_executable(agent
  src/agent.cpp
  # Agents
  src/Agent/Pegasus.cc
  # Policies
  src/Policy/NeuralNetwork.cpp
)

target_link_libraries(agent rlcommon ${catkin_LIBRARIES})
add_dependencies(agent rl_common_generate_messages_cpp)

## Mark executables and/or libraries for installation
install(TARGETS agent
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
