#include <rl_env/trajectory/Trajectory.h>

Trajectory::Trajectory() {
  getting_to_initial_position = true;
}

bool Trajectory::is_within(
  geometry_msgs::Point point1, geometry_msgs::Point point2,
  double ex, double ey, double ez) {

  return (fabs(point1.x - point2.x) < ex &&
          fabs(point1.y - point2.y) < ey &&
          fabs(point1.z - point2.z) < ez);
}
