#include <rl_env/points/PointsBase.h>

PointsBase::PointsBase() {
  axes = "xyz";
}

geometry_msgs::Point PointsBase::get_point(double dir1_val, double dir2_val, double dir3_val) {
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

