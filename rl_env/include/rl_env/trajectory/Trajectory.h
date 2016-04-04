#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include <ros/ros.h>
#include <rl_common/core.hh>
#include <gazebo_msgs/ModelState.h>

class Trajectory {
  /*
  A base trajectory class which is used to implement basic structure of what
  a trajectory means.
  */
public:
  // Any code that resets internal state variables which keeps track of
  // the trajectory.
  virtual void reset() = 0;

  // Computes and returns the current target based on the current state.
  // The current state is defined by the time and state right now.
  virtual gazebo_msgs::ModelState current_target(
    long long timestamp,
    gazebo_msgs::ModelState state) = 0;
};

#endif
