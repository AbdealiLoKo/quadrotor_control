#include <rl_agent/Pegasus.hh>
#include <algorithm>
#include <cfloat>
#include <rl_policy/utils.cpp>

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
  if (left_done) 
    right_value = value;
  else 
    left_value = value;

  update_policy();
  value = 0;
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
  std::vector<float> i(2, 0.1);
  policy = i;
  old_policy = policy;
  new_policy = policy;

  value = 0;
  left_done = false;
  right_done = false;
  parameter = 0;

  return policy.size();
}

std::vector<float> Pegasus::get_action(const std::vector<float> &s) {
  std::vector<float> action(num_outputs);
  action[0] = policy[0] * s[0] + policy[1];
  return action;
}

void Pegasus::update_policy() {
  float epsilon = 0.01;

  // left_done and right_done are flags to indicate if an epsiode has run for which parameter value.
  if (!left_done) {
    left_done = true;
    policy[parameter] = old_policy[parameter] + epsilon;
  }
  else {
    // Left is already completed, do for right
    right_done = true;

    float gradient = (right_value - left_value);
    new_policy[parameter] = old_policy[parameter] + gradient * alpha;

    // Settings for next parameter
    left_done = false;
    right_done = false;

    // Restoring old policy value for particular parameter
    policy[parameter] = old_policy[parameter];
    parameter++;
  
    // Finally processed for all the parameters
    if(parameter == max_parameter) {
      // Debug statements
      std::cout<<"Previous reward "<<right_value<<std::endl;
      std::cout<<"Old policy"<<std::endl;
      print_vector(old_policy);
      std::cout<<"New policy"<<std::endl;
      print_vector(new_policy);

      parameter = 0;
      policy = new_policy;
      old_policy = new_policy;
    } else {
      // Next policy param ready to be updated
      policy[parameter] = old_policy[parameter] - alpha;
    }
  }

  // Do a set_weights(policy) call if using an external policy class
}
