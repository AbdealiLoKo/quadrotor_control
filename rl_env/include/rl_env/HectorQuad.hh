#ifndef _HECTORQUAD_H_
#define _HECTORQUAD_H_

#include <set>
#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <hector_uav_msgs/Altimeter.h>
#include <ros/ros.h>

class HectorQuad: public Environment {
public:
  HectorQuad(Random &rand);

  // Not implemented
  HectorQuad(Random &rand, bool stochastic);

  virtual ~HectorQuad();

  virtual const std::vector<float> &sensation() const;
  virtual float apply(int action);

  virtual bool terminal() const;
  virtual void reset();

  virtual int getNumActions();
  virtual void getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat);
  virtual void getMinMaxReward(float* minR, float* maxR);

  /** Set the state vector (for debug purposes) */
  void setSensation(std::vector<float> newS);

  virtual std::vector<experience> getSeedings();

  /** Get an experience for the given state-action */
  experience getExp(float s0, float s1, int a);

  void zPosCallback(const hector_uav_msgs::Altimeter::ConstPtr& msg);

protected:
  enum quad_action_t {UP_4, UP_3, UP_2, UP_1, ZERO, DOWN_1, DOWN_2, DOWN_3, DOWN_4};
  // 0,1,2,3,4,5,6,7,8,9

  // hardcoded num_actions to be able to calculate action value
  int num_actions = 9;
  ros::Publisher pub;

private:
  const bool noisy;
  Random &rng;
  std::vector<float> s;
  float &zPos;
  float &zVel;
  float reward();
  int zError();
  void refreshState();

  float TARGET;
  float MAX_HEIGHT;
};

#endif
