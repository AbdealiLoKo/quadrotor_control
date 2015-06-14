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
  HectorQuad(Random &rand, Eigen::Vector3d target = Eigen::Vector3d(0, 0, 5));

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

  void gazeboGroundTruth(const nav_msgs::Odometry::ConstPtr& msg);

protected:
  // Actions
  enum quad_action_t {UP, DOWN, STAY};
  // hardcoded num_actions to be able to calculate action value
  int num_actions;
  // Publishers, subscribers and services
  ros::Publisher cmd_vel;
  ros::Subscriber ground_truth;
  ros::ServiceClient reset_world;
  // Stochasticity related variables
  Random &rng;
  // State and positions
  std::vector<float> s;
  Eigen::Vector3d pos, last_pos, vel, last_vel, target_pos;
  // Terminal checker
  int terminal_count;

private:
  float reward();
  void refreshState();
};

#endif
