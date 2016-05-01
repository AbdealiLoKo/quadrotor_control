#ifndef _POINTS_HELIX_H_
#define _POINTS_HELIX_H_

#include <rl_common/core.hh>
#include <rl_env/points/PointsBase.h>

class PointsHelix: public PointsBase {
public:
  // The radius of the circle to make and the constant position in the axis
  // of the circle
  long dir1_radius, dir2_radius, dir1_center, dir2_center, dir3_pos;

  // Frequency and loop count
  long points_per_loop, num_loops;

  std::vector<geometry_msgs::Point> get_points();

  PointsHelix();
};

#endif
