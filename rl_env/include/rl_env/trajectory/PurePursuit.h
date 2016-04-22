#ifndef _PUREPURSUIT_H_
#define _PUREPURSUIT_H_

#include <ros/ros.h>
#include <rl_common/core.hh>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <rl_env/trajectory/Trajectory.h>

class PurePursuit: public Trajectory {
/*
 * Implement PurePursuit tracking
 * Always lock on to a target at a distance LOOKAHEAD
 * from the current position
*/
public:
  double lookahead;
  std::vector<geometry_msgs::Point> points;
  long current_point;

  geometry_msgs::Point initial_position, old_target_for_viz;
  ros::Publisher visualization_publisher;

  PurePursuit(double l);
  virtual void create_waypoints() = 0;

  void reset();

  // Computes and returns the current target based on the current state.
  // The current state is defined by the time and state right now.
  gazebo_msgs::ModelState current_target(long long timestamp,
                                         gazebo_msgs::ModelState state);
  // Functions for visualization
  void visualize_target(geometry_msgs::Point target,
                        geometry_msgs::Vector3 direction);
  void visualize_points();
  gazebo_msgs::ModelState default_target();
  // Internal functions
  double distance_points(geometry_msgs::Point a, geometry_msgs::Point b);
};

#endif
