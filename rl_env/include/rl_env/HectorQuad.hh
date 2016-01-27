#ifndef _HECTORQUAD_H_
#define _HECTORQUAD_H_

#include <rl_common/core.hh>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>

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
  ros::Publisher cmd_vel, motor_pwm;
  ros::ServiceClient reset_world, run_sim, pause_phy, engage, shutdown,
                     list_controllers, load_controller, set_model_state;
  std_srvs::Empty empty_msg;

  // State and positions
  std::vector<float> s;
  gazebo_msgs::ModelState initial, final, current;

  float reward();
  void get_trajectory(long long time_in_steps = -1);
};

#endif
