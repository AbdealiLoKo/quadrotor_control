#ifndef _PURSUIT_CIRCLE_H_
#define _PURSUIT_CIRCLE_H_

#include <rl_common/core.hh>

#include <rl_env/trajectory/Pursuit.h>

class PursuitCircle: public Pursuit {
public:
  // The radius of the circle to make and the constant position in the axis
  // of the circle
  long dir1_radius, dir2_radius, dir1_center, dir2_center, dir3_pos;

  // The axes for (axis of dir1, dir2, dir3). Default: "xyz"
  std::string axes;

  long long steps_per_loop;

  PursuitCircle();
  geometry_msgs::Point compute_lead(long long timestamp);
};

#endif
