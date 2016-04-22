#include <rl_env/trajectory/PurePursuit.h>

PurePursuit::PurePursuit(double l) {
  lookahead = l;

  // Visualize
  ros::NodeHandle nh;
  visualization_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void PurePursuit::reset() {
  points.clear();
  create_waypoints();
  current_point = 0;
  visualize_points();
}

gazebo_msgs::ModelState PurePursuit::current_target(long long timestamp,
  gazebo_msgs::ModelState model_state) {
  geometry_msgs::Point p1 = points[current_point-1];
  geometry_msgs::Point p2 = points[current_point];
  geometry_msgs::Point current = model_state.pose.position;
  geometry_msgs::Vector3 direction;

  // Set default values of target
  gazebo_msgs::ModelState target = default_target();

  if (getting_to_initial_position) {
    target.pose.position = points[current_point];
    if (is_within(model_state.pose.position, points[current_point], 0.25, 0.25, 0.25)) {
      getting_to_initial_position = false;
      current_point += 1;
      ROS_INFO("Reached initial position");
    }
  } else {
    if(current_point == points.size() - 1) {
      target.pose.position = points[points.size()-1];
    } else {
      int i = 0;

      // Check which waypoint to use for following pursuit
      // based on the current position of the point and LOOKAHEAD
      while( distance_points(model_state.pose.position, points[current_point+i]) < lookahead ) {
        i += 1;
        if(current_point + i == points.size() - 1) {
          // Reached final point in the trajectory
          break;
        }
      }

      // Set new one as current checkpoint
      current_point += i;

      // Updated goto points
      p1 = points[current_point-1];
      p2 = points[current_point];

      // Find the projection of the quadrotor
      // position on the trajectory using the dot product
      double projection = (
        (p2.x - p1.x) * (current.x - p1.x) +
        (p2.y - p1.y) * (current.y - p1.y)
      ) / distance_points(p1, p2);

      float lambda = lookahead + projection;

      target.pose.position.x = p1.x + lambda * (p2.x - p1.x);
      target.pose.position.y = p1.y + lambda * (p2.y - p1.y);
      target.pose.position.z = p1.z + lambda * (p2.z - p1.z);

      direction.x = p2.x - p1.x;
      direction.y = p2.y - p1.y;
      direction.z = p2.z - p1.z;
    }
  }

  visualize_target(target.pose.position, direction);
  visualize_points();

  return target;
}

gazebo_msgs::ModelState PurePursuit::default_target() {
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

void PurePursuit::visualize_target(geometry_msgs::Point target,
 geometry_msgs::Vector3 direction) {
  // Demo from http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines
  visualization_msgs::Marker viz_points;

  viz_points.header.frame_id = "/world";
  viz_points.header.stamp = ros::Time::now();
  viz_points.ns = "target_point";
  viz_points.pose.orientation.w = 1.0;
  viz_points.action = visualization_msgs::Marker::ADD;
  viz_points.type = visualization_msgs::Marker::POINTS;

  // Size of points
  viz_points.scale.x = 0.2f;
  viz_points.scale.y = 0.2f;
  viz_points.scale.z = 0.2f;

  // Color
  viz_points.color.r = 1.0f;
  viz_points.color.g = 0.0f;
  viz_points.color.b = 0.0f;
  viz_points.color.a = 1.0f;

  // Add target to it
  viz_points.points.push_back(target);

  visualization_publisher.publish(viz_points);

  // Now publish arrow to denote velocity
  visualization_msgs::Marker viz_arrow;
  viz_arrow.header.frame_id = "/world";
  viz_arrow.header.stamp = ros::Time::now();
  viz_arrow.ns = "target_velocity";
  // viz_arrow.pose.orientation.w = 1.0;
  viz_arrow.action = visualization_msgs::Marker::ADD;
  viz_arrow.type = visualization_msgs::Marker::ARROW;

  // Size of ARROW
  viz_arrow.scale.x = 0.1; // Shaft dia
  viz_arrow.scale.y = 0.2; // Head dia
  // Color
  viz_arrow.color.r = 1.0f;
  viz_arrow.color.g = 0.0f;
  viz_arrow.color.b = 0.0f;
  viz_arrow.color.a = 1.0f;

  // Create second point for arrow
  geometry_msgs::Point point2;
  double norm = sqrt(
    direction.x * direction.x +
    direction.y * direction.y +
    direction.z * direction.z
  );

  point2.x = target.x + direction.x / norm;
  point2.y = target.y + direction.y / norm;
  point2.z = target.z + direction.z / norm;

  viz_arrow.points.push_back(target);
  viz_arrow.points.push_back(point2);

  // std::cout << "Point2:" << point2 << "\n";

  visualization_publisher.publish(viz_arrow);

  old_target_for_viz = target;
}

void PurePursuit::visualize_points() {
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

double PurePursuit::distance_points(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt(
    ( a.z - b.z ) * ( a.z - b.z ) +
    ( a.y - b.y ) * ( a.y - b.y ) +
    ( a.x - b.x ) * ( a.x - b.x )
  );
}