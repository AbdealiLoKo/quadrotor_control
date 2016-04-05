#ifndef _PURSUIT_H_
#define _PURSUIT_H_

#include <ros/ros.h>
#include <rl_common/core.hh>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <rl_env/trajectory/Trajectory.h>

class Pursuit: public Trajectory {
  // A generic class which uses pursuit based trajectories.
public:
  geometry_msgs::Point initial_position, old_target_for_viz;
  ros::Publisher visualization_publisher;
  long long initiating_lag;

  double lead_time;

  Pursuit();

  void reset();

  virtual geometry_msgs::Point compute_lead(long long timestamp) = 0;

  // Computes and returns the current target based on the current state.
  // The current state is defined by the time and state right now.
  gazebo_msgs::ModelState current_target(long long timestamp,
                                         gazebo_msgs::ModelState state);
  void visualize_target(geometry_msgs::Point target,
                        geometry_msgs::Vector3 target_vel);

  gazebo_msgs::ModelState default_target();
};

#endif
