#include <rl_env/points/PointsRectangle.h>

PointsRectangle::PointsRectangle() {
  dir1_corner = -5;
  dir2_corner = -5;
  dir3_corner = 0;
  dir1_side = 10;
  dir2_side = 10;
  num_points_side = 1;
  axes = "xyz";
}

std::vector<geometry_msgs::Point> PointsRectangle::get_points() {
  geometry_msgs::Point wp;
  double dir1_val, dir2_val, dir3_val;

  dir3_val = dir3_corner;

  for(int i=0; i<num_points_side; ++i) {
    dir1_val = dir1_corner + (i*dir1_side)/num_points_side;
    dir2_val = dir2_corner;
    wp = get_point(dir1_val, dir2_val, dir3_val);
    Points.push_back(wp);
  }

  for(int i=0; i<num_points_side; ++i) {
    dir1_val = dir1_corner + dir1_side;
    dir2_val = dir2_corner + (i*dir2_side)/num_points_side;
    wp = get_point(dir1_val, dir2_val, dir3_val);
    Points.push_back(wp);
  }

  for(int i=0; i<num_points_side; ++i) {
    dir1_val = dir1_corner + dir1_side - (i*dir1_side)/num_points_side;
    dir2_val = dir2_corner + dir2_side;
    wp = get_point(dir1_val, dir2_val, dir3_val);
    Points.push_back(wp);
  }

  for(int i=0; i<num_points_side; ++i) {
    dir1_val = dir1_corner;
    dir2_val = dir2_corner + dir2_side - (i*dir2_side)/num_points_side;
    wp = get_point(dir1_val, dir2_val, dir3_val);
    Points.push_back(wp);
  }

  dir1_val = dir1_corner;
  dir2_val = dir2_corner;
  wp = get_point(dir1_val, dir2_val, dir3_val);
  Points.push_back(wp);

  return Points;
}
