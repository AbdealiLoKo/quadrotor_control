#ifndef _HECTORQUAD_H_
#define _HECTORQUAD_H_

#include <unistd.h>
#include <utility>
#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <rl_common/core.hh>

// Messages
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <hector_uav_msgs/MotorPWM.h>
// Services
#include <rl_common/RLRunSim.h>

#define SIMPLE_WAYPOINTS 0
#define PURE_PURSUIT 1
#define ALGORITHM 0

#define TRAIN_PEGASUS true

class HectorQuad: public Environment {
public:
  HectorQuad();

  virtual const std::vector<float> &sensation();
  virtual float apply(std::vector<float> action);

  virtual bool terminal();
  virtual void reset();

protected:
  int n_policy, n_state, n_action;
  int phy_steps;
  long long cur_step; // each step is 0.01 sec

  // Publishers, subscribers and services
  ros::Publisher cmd_vel, motor_pwm, command_twist;
  ros::ServiceClient reset_world, run_sim, pause_phy, engage, shutdown,
                     list_controllers, load_controller, set_model_state;
  std_srvs::Empty empty_msg;

  // State and positions
  std::vector<float> s;
  gazebo_msgs::ModelState initial, final, current;
  gazebo_msgs::ModelState payload_initial, payload_final, payload_current;
  geometry_msgs::Twist prev_vel;

  // Waypoints
  std::vector<std::pair<float, float> > waypoints;
  int curr;

  float reward();
  void get_trajectory(long long time_in_steps = -1);
  int form_waypoints(int k);
  int pure_pursuit(int k);
};

#endif
