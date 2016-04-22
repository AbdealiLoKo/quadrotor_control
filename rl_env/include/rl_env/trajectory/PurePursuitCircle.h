#ifndef _PUREPURSUIT_CIRCLE_H_
#define _PUREPURSUIT_CIRCLE_H_

#include <rl_common/core.hh>

#include <rl_env/trajectory/PurePursuit.h>

class PurePursuitCircle: public PurePursuit {
public:
  // The radius of the circle to make and the constant position in the axis
  // of the circle
  long dir1_radius, dir2_radius, dir1_center, dir2_center, dir3_pos;

  // The axes for (axis of dir1, dir2, dir3). Default: "xyz"
  std::string axes;

  // Frequency and loop count
  long points_per_loop, num_loops;

  PurePursuitCircle(double lookahead);

  void create_waypoints();
};

#endif
