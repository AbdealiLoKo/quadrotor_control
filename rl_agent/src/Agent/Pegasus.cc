#include <rl_agent/Pegasus.hh>
#include <algorithm>

void print_vectorF(std::vector<float> &v) {
  std::cout<<"Size "<<v.size()<<std::endl;
  for(int i=0; i<v.size(); ++i) {
    std::cout<<v[i]<<" ";
  }
  std::cout<<std::endl;
}

Pegasus::Pegasus(int numinputs, int numoutputs, float alpha, float gamma, Random rng):
numinputs(numinputs),
numoutputs(numoutputs),
alpha(alpha),
gamma(gamma),
rng(rng)
{
  std::cout<<"Initializing Pegasus"<<std::endl;
  N = init_policy(numinputs, numoutputs);
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

  // nn = NeuralNetwork(n_in, n_out, 0, 1);
  // int N = nn.get_n_weights();
  // std::cout<<"Using neural network"<<std::endl;
  // old_policy.resize(N);
  // nn.get_weights(old_policy);

  old_policy.resize(2);
  old_policy[0] = 0.1;
  old_policy[1] = 0.1;
  int N = 2;

  policy = old_policy;
  new_policy = old_policy;

  return N;
}

std::vector<float> Pegasus::get_action(const std::vector<float> &s) {
  // Simple policies implemented to visualize the weights
  std::vector<float> action(numoutputs);
  action[0] = (policy[0]*s[0]+policy[1]);

  action[0] = (action[0]>1?1:action[0]);
  action[0] = (action[0]<-1?-1:action[0]);

  // Scale up action
  action[0] *= 10 ;

  // The implementation is using a Neural Network
  // std::vector<float> temp = s;
  // print_vectorF(action);
  // nn.value(temp, action);
  // print_vectorF(action);
  // // action[0] = (action[0]>1?1:action[0]);
  // // action[0] = (action[0]<-1?-1:action[0]);

  // action[0] = (action[0]-0.5)*100;
  // std::cout<<"Action "<<action[0]<<std::endl;

  return action;
}

int Pegasus::first_action(const std::vector<float> &s) {
  // Conversion from action back to the action values from the environment is painful for a generic case
  std::vector<float> action = get_action(s);
  return int(action[0]);
}

int Pegasus::next_action(float r, const std::vector<float> &s) {
  // To us, only the final reward matters from the episode for finding the best policy
  episode_reward = gamma*episode_reward + r;

  std::vector<float> action = get_action(s);
  return int(action[0]);
 }

void Pegasus::last_action(float r) {
  // std::cout<<"Got last action"<<std::endl;
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
  else if (!right_done)
    right_done = true;

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
      std::cout<<"Found new policy"<<std::endl;
      std::cout<<"Previous reward "<<right_value<<std::endl;
      print_vectorF(old_policy);
      parameter_no = 0;
      policy = new_policy;
      old_policy = new_policy;
      print_vectorF(policy);
    } else {
      // Next policy param ready to be updated
      policy[parameter_no] = old_policy[parameter_no] - alpha;
    }
  } else if(left_done) {
    policy[parameter_no] += 2 * alpha;
  }

  nn.set_weights(policy);
}
