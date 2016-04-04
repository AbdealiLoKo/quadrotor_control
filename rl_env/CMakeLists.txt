cmake_minimum_required(VERSION 2.8.3)
project(rl_env)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf gazebo_msgs std_srvs
             rl_common)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES env_hectorquad_world
  CATKIN_DEPENDS roscpp std_msgs tf rl_common gazebo_msgs std_srvs
  DEPENDS Eigen gazebo
)

add_executable(env
  src/env.cpp

  src/Env/HectorQuad.cc

  # Trajectories
  src/Trajectory/Trajectory.cpp
  src/Trajectory/Waypoints.cpp
  src/Trajectory/WaypointsCircle.cpp
  src/Trajectory/Pursuit.cpp
  src/Trajectory/PursuitCircle.cpp
)

add_library(env_hectorquad_world
  src/Env/HectorQuad/world.cc
)

target_link_libraries(env rlcommon ${catkin_LIBRARIES})
add_dependencies(env rl_common_generate_messages_cpp)

target_link_libraries(env_hectorquad_world ${GAZEBO_LIBRARIES})
add_dependencies(env_hectorquad_world rl_common_generate_messages_cpp)

## Mark executables and/or libraries for installation
install(TARGETS env env_hectorquad_world
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
