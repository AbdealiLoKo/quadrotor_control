cmake_minimum_required(VERSION 2.8.3)
project(rl_agent)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rl_msgs tf rl_common)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES agentlib
    CATKIN_DEPENDS roscpp std_msgs rl_msgs tf rl_common
    DEPENDS
)

add_library(agentlib
  src/Agent/DiscretizationAgent.cc
  src/Agent/QLearner.cc
  src/Agent/ModelBasedAgent.cc
  src/Agent/SavedPolicy.cc
  src/Agent/Dyna.cc
  src/Agent/Sarsa.cc
  src/Models/FactoredModel.cc
  src/Models/M5Tree.cc
  src/Models/LinearSplitsTree.cc
  src/Models/C45Tree.cc
  src/Models/Stump.cc
  src/Models/MultipleClassifiers.cc
  src/Models/ExplorationModel.cc
  src/Models/RMaxModel.cc
  src/Models/SepPlanExplore.cc
  src/Planners/ValueIteration.cc
  src/Planners/PolicyIteration.cc
  src/Planners/PrioritizedSweeping.cc
  src/Planners/ETUCT.cc
  src/Planners/PO_ParallelETUCT.cc
  src/Planners/ParallelETUCT.cc
  src/Planners/PO_ETUCT.cc
  src/Planners/MBS.cc
  src/newmat/newmat1.cc
  src/newmat/newmat2.cc
  src/newmat/newmat3.cc
  src/newmat/newmat4.cc
  src/newmat/newmat5.cc
  src/newmat/newmat6.cc
  src/newmat/newmat7.cc
  src/newmat/newmat8.cc
  src/newmat/newmatex.cc
  src/newmat/bandmat.cc
  src/newmat/submat.cc
  src/newmat/myexcept.cc
  src/newmat/cholesky.cc
  src/newmat/evalue.cc
  src/newmat/fft.cc
  src/newmat/hholder.cc
  src/newmat/jacobi.cc
  src/newmat/newfft.cc
  src/newmat/sort.cc
  src/newmat/svd.cc
  src/newmat/newmatrm.cc
  src/newmat/newmat9.cc
)

add_executable(agent
  src/agent.cpp
)

target_link_libraries(agent agentlib ${catkin_LIBRARIES})

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

