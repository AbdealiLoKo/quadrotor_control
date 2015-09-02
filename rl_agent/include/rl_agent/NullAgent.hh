/** \File
    Null agent. Doesnt do anything. */

#ifndef _NULL_AGENT_HH_
#define _NULL_AGENT_HH_

#include <rl_common/core.hh>

#include <vector>

class NullAgent: public Agent {
public:
  NullAgent() {}

  ~NullAgent() {}

  int first_action(const std::vector<float> &s) {return 0;}
  int next_action(float r, const std::vector<float> &s) {return 0;}
  void last_action(float r) {}
  void setDebug(bool d) {}
  void seedExp(std::vector<experience>) {}
  void savePolicy(const char* filename) {}
};

#endif
