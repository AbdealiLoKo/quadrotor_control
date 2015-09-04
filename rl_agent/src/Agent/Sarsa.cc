#include <rl_agent/Sarsa.hh>
#include <algorithm>

Sarsa::Sarsa(int numactions, float gamma,
             float initialvalue, float alpha, float ep, float lambda,
             Random rng):
  numactions(numactions), gamma(gamma),
  initialvalue(initialvalue), alpha(alpha),
  epsilon(ep), lambda(lambda),
  rng(rng)
{
  currentq = NULL;
}

int Sarsa::first_action(const std::vector<float> &s) {
  // clear all eligibility traces
  for (std::map<state_t, std::vector<float> >::iterator i = eligibility.begin();
       i != eligibility.end(); i++) {

    std::vector<float> & elig_s = (*i).second;
    for (int j = 0; j < numactions; j++) {
      elig_s[j] = 0.0;
    }
  }

  // Get action values
  state_t si = canonicalize(s);
  std::vector<float> &Q_s = Q[si];

  // Choose an action
  const std::vector<float>::iterator a =
    rng.uniform() < epsilon
    ? Q_s.begin() + rng.uniformDiscrete(0, numactions - 1) // Choose randomly
    : random_max_element(Q_s.begin(), Q_s.end()); // Choose maximum

  // set eligiblity to 1
  std::vector<float> &elig_s = eligibility[si];
  elig_s[a-Q_s.begin()] = 1.0;

  return a - Q_s.begin();
}

int Sarsa::next_action(float r, const std::vector<float> &s) {
  // Get action values
  state_t st = canonicalize(s);
  std::vector<float> &Q_s = Q[st];
  const std::vector<float>::iterator max =
    random_max_element(Q_s.begin(), Q_s.end());

  // Choose an action
  const std::vector<float>::iterator a =
    rng.uniform() < epsilon
    ? Q_s.begin() + rng.uniformDiscrete(0, numactions - 1)
    : max;

  // Update value for all with positive eligibility
  for (std::map<state_t, std::vector<float> >::iterator i = eligibility.begin();
       i != eligibility.end(); i++) {

    state_t si = (*i).first;
    std::vector<float> & elig_s = (*i).second;
    for (int j = 0; j < numactions; j++) {
      if (elig_s[j] > 0.0) {
        // update
        Q[si][j] += alpha * elig_s[j] * (r + gamma * (*a) - Q[si][j]);
        elig_s[j] *= lambda;
      }
    }
  }

  // Set elig to 1
  eligibility[st][a-Q_s.begin()] = 1.0;
  return a - Q_s.begin();
}

void Sarsa::last_action(float r) {
  // Update value for all with positive eligibility
  for (std::map<state_t, std::vector<float> >::iterator i = eligibility.begin();
       i != eligibility.end(); i++) {

    state_t si = (*i).first;
    std::vector<float> & elig_s = (*i).second;
    for (int j = 0; j < numactions; j++) {
      if (elig_s[j] > 0.0) {
        // update
        Q[si][j] += alpha * elig_s[j] * (r - Q[si][j]);
        elig_s[j] = 0.0;
      }
    }
  }
}

Sarsa::state_t Sarsa::canonicalize(const std::vector<float> &s) {
  const std::pair<std::set<std::vector<float> >::iterator, bool> result =
    statespace.insert(s);
  state_t retval = &*result.first; // Dereference iterator then get pointer
  if (result.second) { // s is new, so initialize Q(s,a) for all a
    std::vector<float> &Q_s = Q[retval];
    Q_s.resize(numactions,initialvalue);
    std::vector<float> &elig = eligibility[retval];
    elig.resize(numactions,0);
  }
  return retval;
}

std::vector<float>::iterator
Sarsa::random_max_element(std::vector<float>::iterator start,
                          std::vector<float>::iterator end) {
  std::vector<float>::iterator max = std::max_element(start, end);
  int n = std::count(max, end, *max);
  if (n > 1) {
    n = rng.uniformDiscrete(1, n);
    while (n > 1) {
      max = std::find(max + 1, end, *max);
      --n;
    }
  }
  return max;
}
