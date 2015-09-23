#ifndef _HECTORQUAD_H_
#define _HECTORQUAD_H_

#include <Eigen/Geometry>
#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <ros/ros.h>
#include <std_srvs/Empty.h>


class HectorQuad: public Environment {
public:
  HectorQuad(Random &rand);

  virtual const std::vector<float> &sensation();
  virtual float apply(std::vector<float> action);

  virtual bool terminal();
  virtual void reset();

protected:
  int phy_steps;
  long long cur_step; // each step is 0.01 sec

  // Publishers, subscribers and services
  ros::Publisher cmd_vel, motor_pwm;
  ros::ServiceClient reset_world, run_sim, pause_phy, engage, shutdown,
                     list_controllers, load_controller;
  std_srvs::Empty empty_msg;

  // Stochasticity related variables
  Random &rng;

  // State and positions
  std::vector<float> s;
  Eigen::Vector3d target_pos;

  float reward();
  void get_trajectory(long long steps = -1);
};

#endif
