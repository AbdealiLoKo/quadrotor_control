#include <rl_env/trajectory/WaypointsCircle.h>

WaypointsCircle::WaypointsCircle(bool _use_checkpoints /*= false*/) :
Waypoints(_use_checkpoints) {
  dir1_radius = 5;
  dir2_radius = 5;
  dir1_center = 0;
  dir2_center = 0;
  dir3_pos = 0;
  axes = "xyz";
  points_per_loop = 16;
  num_loops = 100;
}

void WaypointsCircle::create_waypoints() {
  for (int i = 0; i < 1000; ++i) {
    double dir1_val = dir1_center + dir1_radius * sin(2 * M_PI / points_per_loop * i);
    double dir2_val = dir2_center + dir2_radius * cos(2 * M_PI / points_per_loop * i);
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
    // std::cout << "WP " << i << ": " << wp.x << ", " << wp.y << ", " << wp.z << "\n";
    points.push_back(wp);
    // geometry_msgs::PointStamped geo_point;
    // geo_point.point.x = p.first;
    // geo_point.point.y = p.second;
    // geo_point.point.z = 5;
    // viz_points.publish(geo_point);
  }
}
