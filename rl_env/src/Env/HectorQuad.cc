#include <unistd.h>
#include <geometry_msgs/Twist.h>
#include <rl_msgs/RLRunSim.h>
#include <hector_uav_msgs/MotorPWM.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ListControllers.h>
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
  ros::service::waitForService("/gazebo/pause_physics", -1);
  pause_phy = node.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

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
  // Get state from gazebo
  rl_msgs::RLRunSim msg;
  msg.request.steps = 10; // 1 step = 0.01 sec
  run_sim.call(msg);

  // Convert gazebo's state to internal representation
  s[0] = target_pos(2)-msg.response.pose.position.z;
  s[1] = msg.response.twist.linear.z;

  // std::cout << s << "\n";

  return s;
}

bool HectorQuad::terminal() {
  return(false);
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
  cmd_vel.publish(action_vel);

  // hector_uav_msgs::MotorPWM action_pwm;
  // int pwms[4] = {action, action, action, action};
  // action_pwm.pwm = pwms;
  // motor_pwm.publish(action_pwm);
  return reward();
}

float HectorQuad::reward() {
  return -fabs(s[0]);
}

void HectorQuad::reset() {
  shutdown.call(empty_msg); // shutdown motors
  geometry_msgs::Twist action_vel; // Set velocity to 0
  action_vel.linear.x = 0;
  action_vel.linear.y = 0;
  action_vel.linear.z = 0;
  action_vel.angular.x = 0;
  action_vel.angular.y = 0;
  action_vel.angular.z = 0;

  // Keep checking the status of controller. When it goes into "running"
  // we can start getting actions from the agent.
  // Until then, keep giving a vel of 0 so that it will auto engage.
  std::cout << "HectorQuad : Waiting for controller to engage motors ...\n";
  while(1) {
    controller_manager_msgs::ListControllers list_msg;
    list_controllers.call(list_msg);
    cmd_vel.publish(action_vel);

    usleep(1000);
    if (list_msg.response.controller[0].state == "running")
      break;
  }

  // Reset and pause the world
  // Note: Pause has to be done only after `waitForService` finds the service.
  //       it cannot be done in gazebo as otherwise waitForService hangs.
  pause_phy.call(empty_msg);
  reset_world.call(empty_msg); // Reset the world
}

Eigen::Vector3d HectorQuad::get_pos() {
  // Get state from gazebo
  rl_msgs::RLRunSim msg;
  msg.request.steps = 10; // 1 step = 0.01 sec
  run_sim.call(msg);

  return Eigen::Vector3d(msg.response.pose.position.x, msg.response.pose.position.y, msg.response.pose.position.z);
}

void HectorQuad::set_vel(Eigen::Vector3d v) {
  controller_manager_msgs::ListControllers list_msg;
  list_controllers.call(list_msg);
  assert(list_msg.response.controller[0].state == "running");

  // Send action
  geometry_msgs::Twist action_vel;
  action_vel.linear.x = v[0];
  action_vel.linear.y = v[1];
  action_vel.linear.z = v[2];

  cmd_vel.publish(action_vel);
}
