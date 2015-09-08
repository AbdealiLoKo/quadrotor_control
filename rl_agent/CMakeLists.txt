cmake_minimum_required(VERSION 2.8.3)
project(rl_agent)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rl_msgs tf rl_common rl_policy)
find_package(GSL REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES agentlib
    CATKIN_DEPENDS roscpp std_msgs rl_msgs tf rl_common rl_policy
    DEPENDS
)

add_library(agentlib
  src/Agent/Pegasus.cc
)

add_executable(agent
  src/agent.cpp
)

target_link_libraries(agent
  agentlib
  ${catkin_LIBRARIES}
  ${GSL_LIBRARIES}
  ${GSLCBLAS_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS agent agentlib
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

