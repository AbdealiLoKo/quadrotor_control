#include <rl_env/trajectory/PursuitCircle.h>

PursuitCircle::PursuitCircle() {
  dir1_radius = 5;
  dir2_radius = 5;
  dir1_center = 0;
  dir2_center = 0;
  dir3_pos = 0;
  axes = "xyz";
  steps_per_loop = 40000;
}

geometry_msgs::Point PursuitCircle::compute_lead(long long timestamp) {

  double dir1_val = dir1_center + dir1_radius * sin(2 * M_PI / steps_per_loop * timestamp);
  double dir2_val = dir2_center + dir2_radius * cos(2 * M_PI / steps_per_loop * timestamp);
  double dir3_val = dir3_pos;

  geometry_msgs::Point wp;
  if (axes[0] == 'x') {
    wp.x = dir1_val;
  } else if (axes[0] == 'y') {
    wp.y = dir1_val;
  } else if (axes[0] == 'z') {
    wp.z = dir1_val;
  } else {
    std::cout << "Unexpected value for axis 0\n";
  }

  if (axes[1] == 'x') {
    wp.x = dir2_val;
  } else if (axes[1] == 'y') {
    wp.y = dir2_val;
  } else if (axes[1] == 'z') {
    wp.z = dir2_val;
  } else {
    std::cout << "Unexpected value for axis 1\n";
  }

  if (axes[2] == 'x') {
    wp.x = dir3_val;
  } else if (axes[2] == 'y') {
    wp.y = dir3_val;
  } else if (axes[2] == 'z') {
    wp.z = dir3_val;
  } else {
    std::cout << "Unexpected value for axis 2\n";
  }

  return wp;
}
