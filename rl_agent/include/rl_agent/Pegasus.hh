#ifndef _PEGASUS_HH_
#define _PEGASUS_HH_

#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include "NeuralNetwork.h"

#include <map>
#include <set>
#include <vector>

class Pegasus: public Agent {
public:
  /** Standard constructor
      \param numactions The number of possible actions
  */
  Pegasus(int numinputs, int numoutputs, float alpha, Random rng = Random());

  virtual ~Pegasus();

  virtual int first_action(const std::vector<float> &s);
  virtual int next_action(float r, const std::vector<float> &s);
  virtual void last_action(float r);
  virtual void setDebug(bool d);

  void printState(const std::vector<float> &s);
  int init_policy(int n_i, int n_o);
  float getValue(std::vector<float> state);
  void setValue(int val);
  void newPolicy();
  std::vector<float> getAction(const std::vector<float> &s);

protected:

private:
  Random rng;
  int numoutputs, numinputs;
  float alpha;

  int N, parameter_no;
  float episode_reward;
  float left_value, right_value;
  bool left_done, right_done;
  NeuralNetwork nn;
  float gamma=0.9;

  std::vector<float> old_policy, new_policy;
  std::vector<float> policy;
  
  bool ACTDEBUG;
  bool ELIGDEBUG;
};

#endif
