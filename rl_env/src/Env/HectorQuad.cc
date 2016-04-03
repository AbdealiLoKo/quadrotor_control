#include <rl_env/HectorQuad.hh>

HectorQuad::HectorQuad()
{
  n_action = 4;
  n_state = 8;
  n_policy = 8;

  s.resize(n_state);
  phy_steps = 10;
  cur_step = 0;

  // Set name of model
  initial.model_name = "quadrotor";
  final.model_name = "quadrotor";
  current.model_name = "quadrotor";

  // initial.model_name = "";
  // final.model_name = "";
  // current.model_name = "";

  ros::NodeHandle node;

  // Publishers
  // cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  command_twist = node.advertise<geometry_msgs::TwistStamped>("/command/twist", 5);
  // motor_pwm = node.advertise<hector_uav_msgs::MotorPWM>("/motor_pwm", 5);
  wind = node.advertise<geometry_msgs::Vector3>("/wind", 5);
  syscommand = node.advertise<std_msgs::String>("/syscommand", 5);
  viz_points = node.advertise<geometry_msgs::PointStamped>("/visualize_points", 5);

  // Services
  ros::service::waitForService("/gazebo/reset_world", -1);
  reset_world = node.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
  ros::service::waitForService("/rl_env/run_sim", -1);
  run_sim = node.serviceClient<rl_common::RLRunSim>("/rl_env/run_sim");
  ros::service::waitForService("/gazebo/pause_physics", -1);
  pause_phy = node.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  ros::service::waitForService("/gazebo/set_model_state", -1);
  set_model_state =
    node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // Load the controller needed. If this it done in launch file, it doesnt
  // start on time. So, it needs to be done in sync.
  ros::service::waitForService("/controller_manager/load_controller", -1);
  load_controller =
    node.serviceClient<controller_manager_msgs::LoadController>(
      "/controller_manager/load_controller");
  controller_manager_msgs::LoadController load_msg;
  load_msg.request.name = "controller/twist";
  load_controller.call(load_msg);
  assert(load_msg.response.ok);

  // Check list of controllers loaded
  ros::service::waitForService("/controller_manager/list_controllers", -1);
  list_controllers =
    node.serviceClient<controller_manager_msgs::ListControllers>(
      "/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_msg;
  list_controllers.call(list_msg);
  // The assert assumes that the first controller is twist.
  assert(list_msg.response.controller[0].name == load_msg.request.name);

  ros::service::waitForService("/engage", -1);
  engage = node.serviceClient<std_srvs::Empty>("/engage");
  ros::service::waitForService("/shutdown", -1);
  shutdown = node.serviceClient<std_srvs::Empty>("/shutdown");

  reset();
}

const std::vector<float> &HectorQuad::sensation() {
  prev_vel = current.twist;

  // Get state from gazebo and save to "current" state
  rl_common::RLRunSim msg;
  msg.request.steps = phy_steps;
  cur_step += phy_steps;
  run_sim.call(msg);
  current.pose = msg.response.pose;
  current.twist = msg.response.twist;
  get_trajectory();

  // Set wind for next turn
  if (USE_WIND) {
    double max_wind = 5;
    wind_vel.x += ((double)rand()/RAND_MAX - 0.5) * 2*max_wind * 0.01;
    wind_vel.y += ((double)rand()/RAND_MAX - 0.5) * 2*max_wind * 0.01;
    wind_vel.z += ((double)rand()/RAND_MAX - 0.5) * 2*max_wind * 0.01;
    // truncate wind
    wind_vel.x = std::max(wind_vel.x, -max_wind);
    wind_vel.x = std::min(wind_vel.x, max_wind);
    wind_vel.y = std::max(wind_vel.y, -max_wind);
    wind_vel.y = std::min(wind_vel.y, max_wind);
    wind_vel.z = std::max(wind_vel.z, -max_wind);
    wind_vel.z = std::min(wind_vel.z, max_wind);
    wind.publish(wind_vel);
  }

  // Convert gazebo's state to internal representation
  s[0] = final.pose.position.z - current.pose.position.z;
  s[1] = current.twist.linear.z;
  s[2] = final.pose.position.y - current.pose.position.y;
  s[3] = current.twist.linear.y;
  s[4] = final.pose.position.x - current.pose.position.x;
  s[5] = current.twist.linear.x;
  s[6] = tf::getYaw(final.pose.orientation) - tf::getYaw(current.pose.orientation);
  s[7] = current.twist.angular.z;
  // std::cout << s << std::endl;
  // std::cout << current.twist.linear.x - prev_vel.linear.x << " "
  //           << current.twist.linear.y - prev_vel.linear.y << " "
  //           << current.twist.linear.z - prev_vel.linear.z << "\n";

  // Save the linear acceleration in a file for analysis
  float total_acc = sqrt(current.twist.linear.x * current.twist.linear.x
    + current.twist.linear.y * current.twist.linear.y) -
  sqrt(prev_vel.linear.x * prev_vel.linear.x
    + prev_vel.linear.y * prev_vel.linear.y);
  std::ofstream myfile;
  myfile.open ("acceleration.txt", std::ios::app);
  myfile << total_acc << "\n";
  myfile.close();

  return s;
}

bool HectorQuad::terminal() {
  if (! TRAIN_PEGASUS) {
    return false;
  }

  if (cur_step > 10000) return true;
  return false;
}

float HectorQuad::apply(std::vector<float> action) {
  // The below assert is an "in case" - to check whether the controller
  // is actually engaged before giving the action.
  controller_manager_msgs::ListControllers list_msg;
  list_controllers.call(list_msg);
  assert(list_msg.response.controller[0].state == "running");

  // Send action
  assert(action.size() == n_action);
  geometry_msgs::TwistStamped action_vel;
  action_vel.twist.linear.z = action[0];
  action_vel.twist.linear.y = action[1];
  action_vel.twist.linear.x = action[2];
  action_vel.twist.angular.z = action[3];
  command_twist.publish(action_vel);
  return reward();
}

float HectorQuad::reward() {
  tf::Quaternion curr_quat;
  double curr_roll, curr_pitch, curr_yaw;
  tf::quaternionMsgToTF(current.pose.orientation, curr_quat);
  tf::Matrix3x3(curr_quat).getRPY(curr_roll, curr_pitch, curr_yaw);

  tf::Quaternion final_quat;
  double final_roll, final_pitch, final_yaw;
  tf::quaternionMsgToTF(final.pose.orientation, final_quat);
  tf::Matrix3x3(final_quat).getRPY(final_roll, final_pitch, final_yaw);

  // std::cout << "Yaw : " << curr_yaw << " - " << final_yaw << "\n";
  // std::cout<< final.pose.position.x << " " << current.pose.position.x << "\n";
  return (
    -fabs(final.pose.position.z - current.pose.position.z)
    -fabs(final.pose.position.y - current.pose.position.y)
    -fabs(final.pose.position.x - current.pose.position.x)
    -fabs(final_roll - final_roll) * 10.0
    -fabs(final_pitch - final_pitch) * 10.0
    -fabs(curr_yaw - final_yaw) * 10.0
    // -fabs(pitch) * 10.0
    // -fabs(roll) * 10.0
  );
}

void HectorQuad::reset() {
  shutdown.call(empty_msg); // shutdown motors
  geometry_msgs::TwistStamped action_vel; // Set velocity to 0

  // Keep checking the status of controller. When it goes into "running"
  // we can start getting actions from the agent.
  // Until then, keep giving a vel of 0 so that it will auto engage.
  // std::cout << "HectorQuad : Waiting for controller to engage motors ...\n";
  while(1) {
    controller_manager_msgs::ListControllers list_msg;
    list_controllers.call(list_msg);
    command_twist.publish(action_vel);

    usleep(50);
    if (list_msg.response.controller[0].state == "running")
      break;
  }

  initial.pose.position.x = 0;
  initial.pose.position.y = 0;
  initial.pose.position.z = 0;
  initial.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  initial.twist.linear.x = 0;
  initial.twist.linear.y = 0;
  initial.twist.linear.z = 0;
  initial.twist.angular.x = 0;
  initial.twist.angular.y = 0;
  initial.twist.angular.z = 0;

  wind_vel.x = 0;
  wind_vel.y = 0;
  wind_vel.z = 0;

  // Reset and pause the world
  // Note: Pause has to be done only after `waitForService` finds the service.
  //       it cannot be done in gazebo as otherwise waitForService hangs.
  pause_phy.call(empty_msg);
  reset_world.call(empty_msg);
  cur_step = 0;

  // set initial position programmatically
  gazebo_msgs::SetModelState msg;
  msg.request.model_state = initial;
  assert(set_model_state.call(msg));

  // Reset the trajectory visualizer
  std_msgs::String reset_syscommand;
  reset_syscommand.data = "reset";
  syscommand.publish(reset_syscommand);

  if (ALGORITHM == SIMPLE_WAYPOINTS || ALGORITHM == PURE_PURSUIT) {
    std::vector<std::pair<float, float> > wps;
    for(int i=0; i<1000; ++i) {
      std::pair<float, float> p = std::make_pair(5 * sin(i/50.0), 5 * cos(i/50.0));
      // std::pair<float, float> p = std::make_pair(i/20.0, cos(i/20.0));
      wps.push_back(p);
      // geometry_msgs::PointStamped geo_point;
      // geo_point.point.x = p.first;
      // geo_point.point.y = p.second;
      // geo_point.point.z = 5;
      // viz_points.publish(geo_point);
    }
    waypoints = wps;
  }

  curr = 1;
}

// --------------------------------------------------------------------
// Checkpoints



// --------------------------------------------------------------------
// Waypoints
int HectorQuad::use_waypoints_circle(int wp) {
  /*
   * Checks whether a certain waypoint's plane has been crossed
   * as coming from the previous waypoint
  */
  final.pose.position.x = waypoints[wp].first;
  final.pose.position.y = waypoints[wp].second;

  float d1 = waypoints[wp].first-waypoints[wp-1].first;
  float d2 = waypoints[wp].second-waypoints[wp-1].second;
  float slope = -d1/d2;
  float c = waypoints[wp].second-slope*waypoints[wp].first;

  float p = (waypoints[wp-1].second-slope*waypoints[wp-1].first-c)
            *(current.pose.position.y-slope*current.pose.position.x-c);

  if ( p < 0 ) {
    return wp;
  }

  final.pose.position.x = waypoints[wp+1].first;
  final.pose.position.y =  waypoints[wp+1].second;
  final.pose.position.z = 5;
  return wp + 1;
}


// --------------------------------------------------------------------
// Pure pursuit
int HectorQuad::pure_pursuit_circle(int wp) {
  /*
   * Implement PurePursuit tracking
   * Always lock on to a target at a distance LOOKAHEAD
   * from the current position
  */
  float x1, y1;
  float x = current.pose.position.x;
  float y = current.pose.position.y;
  float theta;
  float LOOKAHEAD = 1;

  float checkpoint_x;
  float checkpoint_y;
  int i = 0;

  // Check which waypoint to use for following pursuit
  // based on the current position of the point and LOOKAHEAD
  while( sqrt( ( y - waypoints[wp+i].second ) * ( y - waypoints[wp+i].second )
      + ( x - waypoints[wp+i].first ) * ( x - waypoints[wp+i].first ) ) < LOOKAHEAD ) {
    i += 1;
    // Check for overflow
    assert(i<waypoints.size());
  }

  x1 = waypoints[wp+i].first;
  y1 = waypoints[wp+i].second;
  theta = atan(abs((y1-y)/(x1-x)));

  // Find the correct angle
  if (y1-y < 0 && x1-x<0)
    theta += 3.14;
  else if (y1-y<0)
    theta = -theta;
  else if (x1-x<0)
    theta = 3.14-theta;

  // Add the lookahead based checkpoint(Linear interpolation)
  checkpoint_x = x + LOOKAHEAD * cos(theta);
  checkpoint_y = y + LOOKAHEAD * sin(theta);

  final.pose.position.x = checkpoint_x;
  final.pose.position.y = checkpoint_y;
  final.pose.position.z = 5;
  return wp + i;
}

void HectorQuad::get_trajectory(long long time_in_steps /* = -1 */) {
  if (time_in_steps == -1) time_in_steps = cur_step;

  final.pose.position.x = 5;
  final.pose.position.y = 5;
  final.pose.position.z = 5;
  final.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
    0, 0, angles::from_degrees(60));

  if ( time_in_steps < 1000 ) {
  } else if (curr <= 1000) {
    if ( ALGORITHM == SIMPLE_WAYPOINTS ) {
      curr = use_waypoints_circle(curr);
    } else if ( ALGORITHM == PURE_PURSUIT ) {
      curr = pure_pursuit_circle(curr);
    }
  }

  final.twist.linear.x = 0;
  final.twist.linear.y = 0;
  final.twist.linear.z = 0;
  final.twist.angular.x = 0;
  final.twist.angular.y = 0;
  final.twist.angular.z = 0;
}
