#ifndef _POINTS_BASE_H_
#define _POINTS_BASE_H_

#include <ros/ros.h>
#include <rl_common/core.hh>
#include <gazebo_msgs/ModelState.h>

class PointsBase {
public:
  // The axes for (axis of dir1, dir2, dir3). Default: "xyz"
  std::string axes;

  PointsBase();
  std::vector<geometry_msgs::Point> Points;
  geometry_msgs::Point get_point(double dir1_val, double dir2_val, double dir3_val);

  virtual std::vector<geometry_msgs::Point> get_points() = 0;
};

#endif
