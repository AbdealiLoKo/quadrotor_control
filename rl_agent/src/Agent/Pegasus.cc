#include <rl_agent/Pegasus.hh>

Pegasus::Pegasus() {
  n_action = 4;
  n_state = 8;
  n_policy = 8;

  policy_stepsize = 0.001;
  discount_factor = 0.90;
  policy_change = 0.01;
  policy_file_name = "policy.txt";

  init_policy();
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
int Pegasus::init_policy() {
  std::vector<float> v_init(n_policy, 0.0);

  std::ifstream f(policy_file_name.c_str());
  if ( f.good() && policy_file_name != "" ) {
    std::istream_iterator<float> start(f), end;
    std::vector<float> file_policy(start, end);
    assert(file_policy.size() == n_policy); // Ensure that the vector is correct size
    v_init = file_policy;
  }
  f.close();

  policy = v_init;
  old_policy = policy;
  new_policy = policy;

  value = 0;
  left_done = false;
  right_done = false;
  parameter = -1;

  std::cout << "Initialized policy = " << policy << "\n";
}

std::vector<float> Pegasus::get_action(const std::vector<float> &s) {
  assert(s.size() == n_state);
  std::vector<float> action(n_action);
  action[0] = policy[0] * s[0] + policy[1] * s[1];
  action[1] = policy[2] * s[2] + policy[3] * s[3];
  action[2] = policy[4] * s[4] + policy[5] * s[5];
  action[3] = policy[6] * s[6] + policy[7] * s[7];
  return action;
}

void Pegasus::update_policy() {
  policy_values[policy] = value;

  if ( parameter == -1 ) {
    // Finding the value of the new_policy first
    std::cout << "Value for new policy = " << policy_values[policy] << "\n\n";
    parameter++;
    policy[parameter] = old_policy[parameter] - policy_change;

  } else if (!left_done) {
    // Left just got over, let's do right
    left_done = true;
    left_value = policy_values[policy];
    policy[parameter] = old_policy[parameter] + policy_change;

  } else {
    // Both (left and right) have completed
    right_done = true;
    right_value = policy_values[policy];

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
    if ( parameter == n_policy ) {
      // Debug statements
      std::cout << "Switching policy -----------------------------------\n";
      std::cout << "Old policy " << old_policy << "\n"
                << "New policy " << new_policy << "\n";
      if ( old_policy == new_policy ) {
        std::cout << "##### FINISHED (same policy got) #####\n";
        exit(0);
      }
      parameter = -1;
      policy = new_policy;
      old_policy = new_policy;

      if ( policy_file_name != "" ) { // Save the current best policy to file
        std::ofstream f(policy_file_name.c_str());
        if ( f.good() ) {
          std::ostream_iterator<float> output_iterator(f, " ");
          std::copy(policy.begin(), policy.end(), output_iterator);
        }
        f.close();
      }

    } else {
      // Next policy param ready to be updated
      policy[parameter] = old_policy[parameter] - policy_change;
    }
  }
  int count = 0;
  while ( policy_values.count(policy) != 0 ) {
    // if policy is already done, skip it.
    update_policy();
    count ++;
    if ( count >= n_policy * 100 ) {
      std::cout << "##### FINISHED (looped policies) #####\n";
      exit(0);
    }
  }
}
