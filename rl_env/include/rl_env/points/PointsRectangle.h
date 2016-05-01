#ifndef _POINTS_RECTANGLE_H_
#define _POINTS_RECTANGLE_H_

#include <rl_common/core.hh>
#include <rl_env/points/PointsBase.h>

class PointsRectangle: public PointsBase {
public:
  double dir1_corner, dir2_corner, dir3_corner;
  double dir1_side, dir2_side;
  int num_points_side;
  std::vector<geometry_msgs::Point> get_points();

  PointsRectangle();
};

#endif
