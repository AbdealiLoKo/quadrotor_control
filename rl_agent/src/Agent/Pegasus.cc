#include <rl_agent/Pegasus.hh>
#include <algorithm>
#include <cfloat>

Pegasus::Pegasus(int numinputs, int numoutputs, float alpha, float gamma, Random rng):
num_inputs(numinputs),
num_outputs(numoutputs),
alpha(alpha),
gamma(gamma),
rng(rng)
{
  max_parameter = init_policy(num_inputs, num_outputs);
}

std::vector<float> Pegasus::first_action(const std::vector<float> &s) {
  std::vector<float> action = get_action(s);
  return action;
}

std::vector<float> Pegasus::next_action(float r, const std::vector<float> &s) {
  // To us, only the final reward matters from the episode for finding the best policy
  value = gamma * value + r;
  std::vector<float> action = get_action(s);
  // std::cout << action[0] << "\n";
  return action;
 }

void Pegasus::last_action(float r) {
  value = gamma * value + r;
  update_policy();
}

// --------------- POLICY ----------------------------

// TODO : Find a list of random numbers as an when required.
// S X A -> S should be a deterministic model
// From the simulator, the model is already deterministic

// Add uncertainities from the random number generated to the model
// Since currently the environments don't have any way to add randomness
// simply modify the state using the random numbers

// Returns No of weights that can be altered
int Pegasus::init_policy(int n_in, int n_out) {
  std::vector<float> i(2, 0.1), z(2, 0);
  value = 0;
  best_value = FLT_MIN;
  policy = i;
  best_policy = i;
  gradient = z;
  parameter = max_parameter;
  return policy.size();
}

std::vector<float> Pegasus::get_action(const std::vector<float> &s) {
  std::vector<float> action(num_outputs);
  action[0] = policy[0] * s[0] + policy[1];
  return action;
}

void Pegasus::update_policy() {
  float epsilon = 0.01;
  if (parameter == max_parameter) {
    best_value = value;
    parameter = 0;
  } else {
    gradient[parameter] = (value - best_value) / epsilon;
    parameter += 1;
  }
  if (parameter == max_parameter) {
    // Update the policy with the gradient
    for (int i = 0; i < policy.size(); i++) {
      best_policy[i] = best_policy[i] - alpha * gradient[i];
    }
    policy = best_policy;
  } else {
    // This block finds the gradient by making a slight change in each param
    policy = best_policy;
    policy[parameter] = policy[parameter] + epsilon;
  }
}
