#include <rl_env/trajectory/Pursuit.h>

Pursuit::Pursuit() {
  lead_time = 1000; // in steps
  initiating_lag = 0;

  // Visualize
  ros::NodeHandle nh;
  visualization_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void Pursuit::reset() {}

gazebo_msgs::ModelState Pursuit::current_target(
  long long timestamp,
  gazebo_msgs::ModelState model_state) {

  // Set default values of target
  gazebo_msgs::ModelState target = default_target();

  if (getting_to_initial_position) { // Compute initial position dynamically
    initial_position = compute_lead(0);
  }
  if (getting_to_initial_position &&
      (abs(model_state.pose.position.x - initial_position.x) > 0.25 ||
       abs(model_state.pose.position.y - initial_position.y) > 0.25 ||
       abs(model_state.pose.position.z - initial_position.z) > 0.25)) {
    target.pose.position = initial_position;
    // ROS_INFO("Waiting for initial position to be reached");

  } else {
    if (getting_to_initial_position) {
      initiating_lag = timestamp;
      getting_to_initial_position = false;
    }
    target.pose.position = compute_lead(timestamp - initiating_lag);
  }
  visualize_target(target.pose.position, target.twist.linear);
  return target;
}

gazebo_msgs::ModelState Pursuit::default_target() {
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

void Pursuit::visualize_target(geometry_msgs::Point target,
                               geometry_msgs::Vector3 target_vel) {
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

  // Green color
  viz_points.color.r = 1.0f;
  viz_points.color.g = 0.0f;
  viz_points.color.b = 0.0f;
  viz_points.color.a = 1.0f;

  // Add target to it
  viz_points.points.push_back(target);

  visualization_publisher.publish(viz_points);
}
