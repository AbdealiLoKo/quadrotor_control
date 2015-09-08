#include <rl_agent/Pegasus.hh>
#include <algorithm>

Pegasus::Pegasus(int numinputs, int numoutputs, float alpha, float gamma, Random rng):
numinputs(numinputs),
numoutputs(numoutputs),
alpha(alpha),
gamma(gamma),
rng(rng)
{
  init_policy(numinputs, numoutputs);
  // Parameters for processing multiple episodes
  parameter_no = 0;
  left_done = false;
  right_done = false;
}

/********************NOT IMPLEMENTED******************************************/
// Find a list of random numbers as an when required.
// S X A -> S should be a deterministic model
// From the simulator, the model is already deterministic

// Add uncertainities from the random number generated to the model
// Since currently the environments don't have any way to add randomness
// simply modify the state using the random numbers

// Returns No of weights that can be altered
int Pegasus::init_policy(int n_in, int n_out) {
  // Our initial policy is randomly initializing the neural network
  // Inputs, Outputs, No of hidden layers, No per hidden layer

  old_policy.resize(2);
  old_policy[0] = 0.1;
  old_policy[1] = 0.1;
  policy = old_policy;
  new_policy = old_policy;

  return 2; // number of parameters
}

std::vector<float> Pegasus::get_action(const std::vector<float> &s) {
  // Simple policies implemented to visualize the weights
  std::vector<float> action(numoutputs);
  action[0] = (policy[0]*s[0]+policy[1]);

  return action;
}

std::vector<float> Pegasus::first_action(const std::vector<float> &s) {
  std::vector<float> action = get_action(s);
  return action;
}

std::vector<float> Pegasus::next_action(float r, const std::vector<float> &s) {
  // To us, only the final reward matters from the episode for finding the best policy
  episode_reward = gamma*episode_reward + r;
  std::vector<float> action = get_action(s);
  // std::cout << action[0] << "\n";
  return action;
 }

void Pegasus::last_action(float r) {
  episode_reward = gamma*episode_reward + r;
  if (left_done) { // Get the new value from the agent
    right_value = episode_reward;
  }
  else {
    left_value = episode_reward;
  }
  episode_reward = 0;
  update_policy();
}

void Pegasus::update_policy() {
  // Checks if policy needs to be updated
  if (!left_done) {
    left_done = true;
  }
  else if (!right_done) {
    right_done = true;
  }

  if (right_done) {
    float gradient = (right_value - left_value);
    // Change alpha to increase/reduce speed of gradient descent
    new_policy[parameter_no] = old_policy[parameter_no] - gradient * alpha;

    left_done = false;
    right_done = false;

    // Restoring old policy value
    policy[parameter_no] = old_policy[parameter_no];
    parameter_no++;

    // Finally processed for all the parameters
    if(parameter_no == N) {
      std::cout << "Found new policy"<<std::endl;
      std::cout << "Previous reward "<<right_value<<std::endl;
      std::cout << old_policy << "\n";
      parameter_no = 0;
      policy = new_policy;
      old_policy = new_policy;
      std::cout << policy << "\n";
    } else {
      // Next policy param ready to be updated
      policy[parameter_no] = old_policy[parameter_no] - alpha;
    }
  } else if(left_done) {
    policy[parameter_no] += 2 * alpha;
  }
}
