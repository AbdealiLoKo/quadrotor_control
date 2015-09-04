#ifndef _PEGASUS_HH_
#define _PEGASUS_HH_

#include <rl_common/Random.h>
#include <rl_common/core.hh>
#include <rl_policy/NeuralNetwork.h>

#include <map>
#include <set>
#include <vector>

class Pegasus: public Agent {
public:
  /** Standard constructor
      \param numinputs The number of possible inputs
      \param numoutputs The number of possible outputs
  */
  Pegasus(int numinputs, int numoutputs, float alpha, float gamma,
          Random rng = Random());

  virtual ~Pegasus() {}

  virtual int first_action(const std::vector<float> &s);
  virtual int next_action(float r, const std::vector<float> &s);
  virtual void last_action(float r);

  int init_policy(int n_i, int n_o);
  void update_policy();
  std::vector<float> get_action(const std::vector<float> &s);

protected:

private:
  Random rng;
  int numoutputs, numinputs;
  float alpha, gamma;

  int N, parameter_no;
  float episode_reward;
  float left_value, right_value;
  bool left_done, right_done;
  NeuralNetwork nn;

  std::vector<float> old_policy, new_policy;
  std::vector<float> policy;
};

#endif
