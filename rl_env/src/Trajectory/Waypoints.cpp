#include <rl_env/trajectory/Waypoints.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

Waypoints::Waypoints(bool _use_checkpoints /*= false*/) {
  time_to_get_to_position = 1000;
  current_point = -1;
  epsilon_plane = 0.3;
  epsilon_x = epsilon_y = 0.3;
  epsilon_z = 1;

  use_checkpoint_condition = _use_checkpoints;

  // Visualize waypoints
  ros::NodeHandle nh;
  visualization_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void Waypoints::reset() {
  points.clear();
  create_waypoints();
  current_point = 1;
  visualize_points();
}

gazebo_msgs::ModelState Waypoints::current_target(
  long long timestamp,
  gazebo_msgs::ModelState model_state) {

  if (current_point == -1) {
    std::cout << "Trajectory has not been reset, waypoints were not created.\n";
  }

  // Set default values of target
  gazebo_msgs::ModelState target = default_target();

  // Wait for initial buffer
  if (getting_to_initial_position &&
      ! is_within(model_state.pose.position, points[0], 0.25, 0.25, 0.25)) {
    target.pose.position = points[0];
    // ROS_INFO("Waiting for initial position to be reached");

  } else {
    if (getting_to_initial_position) {
      // Finished waiting for "initial state"
      getting_to_initial_position = false;
    }

    // Calculate derivative
    geometry_msgs::Vector3 derivative;
    derivative.x = points[current_point].x - points[current_point-1].x;
    derivative.y = points[current_point].y - points[current_point-1].y;
    derivative.z = points[current_point].z - points[current_point-1].z;

    if (use_checkpoint_condition) {
      if (is_within(model_state.pose.position, points[current_point],
                    epsilon_x, epsilon_y, epsilon_z)) {
        current_point += 1;
      }
    } else {
      // Make a plane a little in front of the current waypoint as we need
      // to truncate the policy before.
      geometry_msgs::Point plane_point;
      plane_point = points[current_point];
      plane_point.x -= derivative.x * epsilon_plane;
      plane_point.y -= derivative.y * epsilon_plane;
      plane_point.z -= derivative.z * epsilon_plane;

      // Find equation of plane: a * (x - x0) + b * (y - y0) + c * (z - z0) = 0
      double cur_side = equation_plane(derivative,
                                       plane_point,
                                       model_state.pose.position);
      double old_side = equation_plane(derivative,
                                       plane_point,
                                       points[current_point-1]);

      // Move to next point if plane was passed. We check this by checking if
      // both the last waypoint and current position are on the same side of the
      // plane.
      if ( cur_side * old_side <= 0 && current_point != points.size()-1 ) {
        current_point += 1;
      }
      visualize_plane(plane_point, derivative);
    }

    target.pose.position = points[current_point];
  }
  visualize_points();
  visualize_target(target.pose.position);
  return target;
}

double Waypoints::equation_plane(geometry_msgs::Vector3 normal,
                                 geometry_msgs::Point point1,
                                 geometry_msgs::Point point2) {
  // equation of plane: a * (x - x0) + b * (y - y0) + c * (z - z0) = 0
  return (normal.x * (point1.x - point2.x) +
          normal.y * (point1.y - point2.y) +
          normal.z * (point1.z - point2.z));
}

gazebo_msgs::ModelState Waypoints::default_target() {
  gazebo_msgs::ModelState target;

  target.pose.position.x = 0;
  target.pose.position.y = 0;
  target.pose.position.z = 0;
  target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
    angles::from_degrees(0),
    angles::from_degrees(0),
    angles::from_degrees(0));
  target.twist.linear.x = 0;
  target.twist.linear.y = 0;
  target.twist.linear.z = 0;
  target.twist.angular.x = 0;
  target.twist.angular.y = 0;
  target.twist.angular.z = 0;

  return target;
}

void Waypoints::visualize_points() {
  // Demo from http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
  visualization_msgs::Marker viz_points;

  viz_points.header.frame_id = "/world";
  viz_points.header.stamp = ros::Time::now();
  viz_points.ns = "all_points";
  viz_points.pose.orientation.w = 1.0;
  viz_points.action = visualization_msgs::Marker::ADD;
  viz_points.type = visualization_msgs::Marker::POINTS;

  // Size of points
  viz_points.scale.x = 0.2;
  viz_points.scale.y = 0.2;

  // Green color
  viz_points.color.r = 0.0f;
  viz_points.color.g = 0.0f;
  viz_points.color.b = 1.0f;
  viz_points.color.a = 1.0f;

  // Add waypoints to it
  viz_points.points = points;

  visualization_publisher.publish(viz_points);
}

void Waypoints::visualize_plane(geometry_msgs::Point target, geometry_msgs::Vector3 vec) {
  // Demo from http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
  visualization_msgs::Marker viz_points, viz_points2;
  double plane_size = 0.6;

  viz_points.header.frame_id = "/world";
  viz_points.header.stamp = ros::Time::now();
  viz_points.ns = "target_plane";
  viz_points.pose.orientation.w = 1.0;
  viz_points.action = visualization_msgs::Marker::ADD;
  viz_points.type = visualization_msgs::Marker::LINE_STRIP;

  // Size of points
  viz_points.scale.x = 0.2;
  viz_points.scale.y = 0.2;

  // Green color
  viz_points.color.r = 0.0f;
  viz_points.color.g = 1.0f;
  viz_points.color.b = 0.0f;
  viz_points.color.a = 1.0f;

  geometry_msgs::Point p1, p2, p3, p4;
  p1 = p2 = p3 = p4 = target;

  // Default plane
  Eigen::Vector3d A(0, 0, 1);
  Eigen::Vector3d B;
  tf::vectorMsgToEigen(vec, B);

  Eigen::Matrix3d R;
  Eigen::Vector3d p, temp;

  // Find the rotation matrix
  R = Eigen::Quaterniond().setFromTwoVectors(A, B);

  // Transform points around the target
  // Rotation step
  p = R * Eigen::Vector3d(plane_size, plane_size, 0);
  tf::pointMsgToEigen(p1, temp);
  // Translation
  tf::pointEigenToMsg(p+temp, p1);

  p = R * Eigen::Vector3d(-plane_size, plane_size, 0);
  tf::pointMsgToEigen(p2, temp);
  tf::pointEigenToMsg(p+temp, p2);

  p = R * Eigen::Vector3d(-plane_size, -plane_size, 0);
  tf::pointMsgToEigen(p3, temp);
  tf::pointEigenToMsg(p+temp, p3);

  p = R * Eigen::Vector3d(plane_size, -plane_size, 0);
  tf::pointMsgToEigen(p4, temp);
  tf::pointEigenToMsg(p+temp, p4);

  viz_points.points.push_back(p1);
  viz_points.points.push_back(p2);
  viz_points.points.push_back(p3);
  viz_points.points.push_back(p4);
  viz_points.points.push_back(p1);
  visualization_publisher.publish(viz_points);
}

void Waypoints::visualize_target(geometry_msgs::Point target) {
  // Demo from http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
  visualization_msgs::Marker viz_points;

  viz_points.header.frame_id = "/world";
  viz_points.header.stamp = ros::Time::now();
  viz_points.ns = "target_point";
  viz_points.pose.orientation.w = 1.0;
  viz_points.action = visualization_msgs::Marker::ADD;
  viz_points.type = visualization_msgs::Marker::POINTS;

  // Size of points
  viz_points.scale.x = 0.2;
  viz_points.scale.y = 0.2;

  // Red color
  viz_points.color.r = 1.0f;
  viz_points.color.g = 0.0f;
  viz_points.color.b = 0.0f;
  viz_points.color.a = 1.0f;

  // Add target to it
  viz_points.points.push_back(target);

  visualization_publisher.publish(viz_points);
}
