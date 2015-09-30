#include <rl_agent/Pegasus.hh>
#include <algorithm>
#include <cfloat>

Pegasus::Pegasus(int nstate, int naction, Random rng):
n_state(nstate),
n_action(naction),
rng(rng)
{
  n_action = 2;
  n_state = 4;
  policy_stepsize = 0.02;
  discount_factor = 0.90;
  policy_change = 0.01;

  max_parameter = init_policy(n_state, n_action);
}

std::vector<float> Pegasus::first_action(const std::vector<float> &s) {
  std::vector<float> action = get_action(s);
  return action;
}

std::vector<float> Pegasus::next_action(float r, const std::vector<float> &s) {
  // To us, only the final reward matters from the episode for finding the best policy
  value = discount_factor * value + r;
  std::vector<float> action = get_action(s);
  // std::cout << action[0] << " " << value << " " << r << "\n";
  return action;
 }

void Pegasus::last_action(float r) {
  value = discount_factor * value + r;
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
int Pegasus::init_policy(int n_state, int n_action) {
  std::vector<float> v_init(n_state);
  for (int i = 0; i < v_init.size(); i++) {
    v_init[i] = 0.00;
  }
  policy = v_init;
  old_policy = policy;
  new_policy = policy;

  value = 0;
  left_done = false;
  right_done = false;
  parameter = 0;

  return policy.size();
}

std::vector<float> Pegasus::get_action(const std::vector<float> &s) {
  std::vector<float> action(n_action);
  action[0] = policy[0] * s[0] + policy[1] * s[1];
  action[1] = policy[2] * s[2] + policy[3] * s[3];
  return action;
}

void Pegasus::update_policy() {
  // left_done and right_done are flags to indicate if an epsiode has run for which parameter value.
  if (!left_done) {
    left_done = true;
    policy[parameter] = old_policy[parameter] + policy_change;
  }
  else {
    // Left is already completed, do for right
    right_done = true;

    std::cout << "Value = (" << left_value << ", " << right_value << ")\n"
                << "for Policy = " << policy << "\n";


    float gradient = (right_value - left_value);
    new_policy[parameter] = old_policy[parameter] + gradient * policy_stepsize;

    // Settings for next parameter
    left_done = false;
    right_done = false;

    // Restoring old policy value for particular parameter
    policy[parameter] = old_policy[parameter];
    parameter++;

    // Finally processed for all the parameters
    if(parameter == max_parameter) {
      // Debug statements
      std::cout << "Switching policy -----------------------------------\n";
      std::cout << "Old policy " << old_policy << "\n"
                << "New policy " << new_policy << "\n\n";
      if ( old_policy == new_policy ) {
        std::cout << "########## FINISHED ##########\n";
        exit(0);
      }
      parameter = 0;
      policy = new_policy;
      old_policy = new_policy;
    } else {
      // Next policy param ready to be updated
      policy[parameter] = old_policy[parameter] - policy_change;
    }
  }

  // Do a set_weights(policy) call if using an external policy class
}
