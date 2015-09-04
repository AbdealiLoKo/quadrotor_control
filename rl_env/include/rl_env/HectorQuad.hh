#ifndef _HECTORQUAD_H_
#define _HECTORQUAD_H_

#include <Eigen/Geometry>
#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <ros/ros.h>
#include <std_srvs/Empty.h>


class HectorQuad: public Environment {
public:
  HectorQuad(Random &rand,
             Eigen::Vector3d target = Eigen::Vector3d(0, 0, 5));

  virtual const std::vector<float> &sensation();
  virtual float apply(float action);

  virtual bool terminal();
  virtual void reset();

protected:
  // Publishers, subscribers and services
  ros::Publisher cmd_vel;
  ros::ServiceClient reset_world, run_sim;
  std_srvs::Empty empty_msg;
  // Stochasticity related variables
  Random &rng;
  // State and positions
  std::vector<float> s;
  Eigen::Vector3d target_pos;

  float reward();
  void refreshState();
};

#endif
