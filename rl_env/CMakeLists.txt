cmake_minimum_required(VERSION 2.8.3)
project(rl_env)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rl_msgs tf rl_common)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES envlib
    CATKIN_DEPENDS roscpp std_msgs rl_msgs tf rl_common
    DEPENDS
)

add_library(envlib
  src/Env/tworooms.cc
  src/Env/taxi.cc
  src/Env/MountainCar.cc
  src/Env/FuelRooms.cc
  src/Env/CartPole.cc
  src/Env/fourrooms.cc
  src/Env/RobotCarVel.cc
  src/Env/energyrooms.cc
  src/Env/gridworld.cc
  src/Env/stocks.cc
  src/Env/LightWorld.cc
  src/Env/HectorQuad.cc
)

add_executable(env
  src/env.cpp
)

target_link_libraries(envlib ${catkin_LIBRARIES})
target_link_libraries(env envlib ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS env envlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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


