#include <geometry_msgs/Twist.h>
#include <rl_msgs/RLRunSim.h>
#include <hector_uav_msgs/MotorPWM.h>
#include <controller_manager_msgs/LoadController.h>
#include <rl_env/HectorQuad.hh>

// Random initialization of position
HectorQuad::HectorQuad(Random &rand,
                       Eigen::Vector3d target/* = Eigen::Vector3d(0, 0, 5)*/):
s(2),
rng(rand),
target_pos(target)
{
  ros::NodeHandle node;

  // Publishers
  cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  motor_pwm = node.advertise<hector_uav_msgs::MotorPWM>("/motor_pwm", 5);

  // Services
  ros::service::waitForService("/gazebo/reset_world", -1);
  reset_world = node.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
  ros::service::waitForService("/rl_env/run_sim", -1);
  run_sim = node.serviceClient<rl_msgs::RLRunSim>("/rl_env/run_sim");

  // Load the controller needed. If this it done in launch file, it doesnt
  // start on time. So, it needs to be done in sync.
  ros::service::waitForService("/controller_manager/load_controller", -1);
  ros::ServiceClient load_controller =
    node.serviceClient<controller_manager_msgs::LoadController>(
      "/controller_manager/load_controller");
  controller_manager_msgs::LoadController msg;
  msg.request.name = "controller/twist";
  load_controller.call(msg);
  assert(msg.response.ok);

  // Engage motors so that controller can control them from the start
  ros::service::waitForService("/engage", -1);
  ros::ServiceClient engage = node.serviceClient<std_srvs::Empty>("/engage");
  engage.call(empty_msg);

  // Pause the world and reset
  // Note: Pause has to be done only after `waitForService` finds the service.
  //       it cannot be done in gazebo as otherwise waitForService hangs.
  ros::service::waitForService("/gazebo/pause_physics", -1);
  ros::ServiceClient pause_phy = node.serviceClient<std_srvs::Empty>(
    "/gazebo/pause_physics");
  pause_phy.call(empty_msg);

  reset();
}

const std::vector<float> &HectorQuad::sensation() {
  // Get state from gazebo
  rl_msgs::RLRunSim msg;
  msg.request.steps = 1;
  run_sim.call(msg);

  // Convert gazebo's state to internal representation
  s[0] = msg.response.pose.position.z;
  s[1] = msg.response.twist.linear.z;

  return s;
}

bool HectorQuad::terminal() {
  return(false);
}

float HectorQuad::apply(float action) {
  geometry_msgs::Twist action_vel;
  action_vel.linear.z = action;
  cmd_vel.publish(action_vel);

  // hector_uav_msgs::MotorPWM action_pwm;
  // int pwms[4] = {action, action, action, action};
  // action_pwm.pwm = pwms;
  // motor_pwm.publish(action_pwm);
  return reward();
}

float HectorQuad::reward() {
  return -fabs(target_pos(2) - s[0]);
}

void HectorQuad::reset() {
  reset_world.call(empty_msg); // Reset the world
}
