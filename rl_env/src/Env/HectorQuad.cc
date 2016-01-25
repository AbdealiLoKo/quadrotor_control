#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <rl_common/RLRunSim.h>
#include <hector_uav_msgs/MotorPWM.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <rl_env/HectorQuad.hh>

HectorQuad::HectorQuad():
s(6)
{
  phy_steps = 10;
  cur_step = 0;

  // Set name of model
  initial.model_name = "quadrotor";
  final.model_name = "quadrotor";
  current.model_name = "quadrotor";

  ros::NodeHandle node;

  // Publishers
  cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  // motor_pwm = node.advertise<hector_uav_msgs::MotorPWM>("/motor_pwm", 5);

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
  // Get state from gazebo and save to "current" state
  rl_common::RLRunSim msg;
  msg.request.steps = phy_steps;
  cur_step += phy_steps;
  run_sim.call(msg);
  current.pose = msg.response.pose;
  current.twist = msg.response.twist;

  get_trajectory();

  // Convert gazebo's state to internal representation
  s[0] = final.pose.position.z - current.pose.position.z;
  s[1] = current.twist.linear.z;
  s[2] = final.pose.position.y - current.pose.position.y;
  s[3] = current.twist.linear.y;

  // std::cout << s << "\n";

  return s;
}

bool HectorQuad::terminal() {
  if (cur_step > 10000) return true;
  return false;
}

float HectorQuad::apply(std::vector<float> action) {
  // The below assert is an "in case" - to check whether the controller
  // is actually engaged before giving the cmd_vel.
  controller_manager_msgs::ListControllers list_msg;
  list_controllers.call(list_msg);
  assert(list_msg.response.controller[0].state == "running");

  // Send action
  geometry_msgs::Twist action_vel;
  action_vel.linear.z = action[0];
  action_vel.linear.y = action[1];
  cmd_vel.publish(action_vel);
  return reward();
}

float HectorQuad::reward() {
  return (
    -fabs(final.pose.position.z - current.pose.position.z)
    -fabs(final.pose.position.x - current.pose.position.x)
    -fabs(final.pose.position.y - current.pose.position.y)
  );
}

void HectorQuad::reset() {
  shutdown.call(empty_msg); // shutdown motors
  geometry_msgs::Twist action_vel; // Set velocity to 0

  // Keep checking the status of controller. When it goes into "running"
  // we can start getting actions from the agent.
  // Until then, keep giving a vel of 0 so that it will auto engage.
  // std::cout << "HectorQuad : Waiting for controller to engage motors ...\n";
  while(1) {
    controller_manager_msgs::ListControllers list_msg;
    list_controllers.call(list_msg);
    cmd_vel.publish(action_vel);

    usleep(1000);
    if (list_msg.response.controller[0].state == "running")
      break;
  }

  initial.pose.position.x = 0;
  initial.pose.position.y = 0;
  initial.pose.position.z = 0;
  initial.pose.orientation.x = 0;
  initial.pose.orientation.y = 0;
  initial.pose.orientation.z = 0;
  initial.pose.orientation.w = 1;

  initial.twist.linear.x = 0;
  initial.twist.linear.y = 0;
  initial.twist.linear.z = 0;
  initial.twist.angular.x = 0;
  initial.twist.angular.y = 0;
  initial.twist.angular.z = 0;

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
}

void HectorQuad::get_trajectory(long long time_in_steps /* = 0 */) {
  if (time_in_steps == -1) time_in_steps = cur_step;

  final.pose.position.x = 0;
  final.pose.position.y = 5;
  final.pose.position.z = 5;
  final.pose.orientation.x = 0;
  final.pose.orientation.y = 0;
  final.pose.orientation.z = 0;
  final.pose.orientation.w = 1;

  final.twist.linear.x = 0;
  final.twist.linear.y = 0;
  final.twist.linear.z = 0;
  final.twist.angular.x = 0;
  final.twist.angular.y = 0;
  final.twist.angular.z = 0;
}
