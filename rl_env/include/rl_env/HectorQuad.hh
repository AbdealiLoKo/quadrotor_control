#ifndef _HECTORQUAD_H_
#define _HECTORQUAD_H_

#include <set>
#include <Eigen/Geometry>
#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class HectorQuad: public Environment {
public:
  HectorQuad(Random &rand, int target = 5);

  // Not implemented
  // HectorQuad(Random &rand, bool stochastic);

  virtual ~HectorQuad();

  virtual const std::vector<float> &sensation();
  virtual float apply(int action);

  virtual bool terminal();
  virtual void reset();

  virtual int getNumActions();
  virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
  virtual void getMinMaxReward(float* minR, float* maxR);

  /** Set the state vector (for debug purposes) */
  void setSensation(std::vector<float> newS);

  virtual std::vector<experience> getSeedings();

  /** Get an experience for the given state-action */
  experience getExp(float s0, float s1, int a);

  void gazeboStateCallback(const nav_msgs::Odometry::ConstPtr& msg);

protected:
  enum quad_action_t {UP, DOWN, STAY};

  // hardcoded num_actions to be able to calculate action value
  int num_actions;
  ros::Publisher cmd_vel;
  ros::Subscriber quadrotor_state;

private:
  const bool noisy;
  Random &rng;
  std::vector<float> s;
  Eigen::Vector3d pos, last_pos, vel, last_vel;
  float reward();
  void refreshState();

  float TARGET;
  float MAX_HEIGHT;
};

#endif
