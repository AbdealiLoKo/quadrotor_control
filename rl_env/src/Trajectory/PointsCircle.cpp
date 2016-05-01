#include <rl_env/points/PointsCircle.h>

PointsCircle::PointsCircle() :
 PointsBase() {
  dir1_radius = 5;
  dir2_radius = 5;
  dir1_center = 0;
  dir2_center = 0;
  dir3_pos = 0;
  axes = "xyz";
  points_per_loop = 20;
  num_loops = 2;
}

std::vector<geometry_msgs::Point> PointsCircle::get_points() {
 for (int i = 0; i < points_per_loop * num_loops; ++i) {
    double dir1_val = dir1_center + dir1_radius * sin(2 * M_PI / points_per_loop * i);
    double dir2_val = dir2_center + dir2_radius * cos(2 * M_PI / points_per_loop * i);
    double dir3_val = dir3_pos;

    geometry_msgs::Point wp = get_point(dir1_val, dir2_val, dir3_val);
    // std::cout << "WP " << i << ": " << wp.x << ", " << wp.y << ", " << wp.z << "\n";
    Points.push_back(wp);
    // geometry_msgs::PointStamped geo_point;
    // geo_point.point.x = p.first;
    // geo_point.point.y = p.second;
    // geo_point.point.z = 5;
    // viz_points.publish(geo_point);
  }
  return Points;
}
