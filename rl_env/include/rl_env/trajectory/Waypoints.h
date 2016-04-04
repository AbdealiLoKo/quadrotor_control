#ifndef _WAYPOINTS_H_
#define _WAYPOINTS_H_

#include <ros/ros.h>
#include <rl_common/core.hh>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#include <rl_env/trajectory/Trajectory.h>

class Waypoints: public Trajectory {
  // A generic class which uses waypoint based trajectories.
  // Waypoints are calculated based on whether the quad passed the plane
  // of the point or not.
public:
  ros::Publisher visualization_publisher;
  std::vector<geometry_msgs::Point> points;
  long current_point;
  long time_to_get_to_position;
  double epsilon_plane;

  // These are all for Checkpoints. This is mainly to reduce duplicate code for
  // Generating checkpoints/waypoints - which has the same code.
  bool use_checkpoint_condition;
  double epsilon_x, epsilon_y, epsilon_z;

  Waypoints(bool _use_checkpoints = false);

  virtual void create_waypoints() = 0;

  void reset();

  // Computes and returns the current target based on the current state.
  // The current state is defined by the time and state right now.
  gazebo_msgs::ModelState current_target(long long timestamp,
                                         gazebo_msgs::ModelState state);
  void visualize_points();
  void visualize_target(geometry_msgs::Point target);

  // Internal functions
  double equation_plane(geometry_msgs::Vector3 normal,
                        geometry_msgs::Point point1,
                        geometry_msgs::Point point2);
  gazebo_msgs::ModelState default_target();
};

#endif
