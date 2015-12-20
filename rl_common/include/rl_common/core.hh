#ifndef _RLCORE_H_
#define _RLCORE_H_

#include <vector>
#include <iostream>
#include <Eigen/Geometry>

#define NINF INT_MIN
#define INF INT_MAX

std::ostream& operator<< (std::ostream& out, const Eigen::Vector3d& v) {
  out << "[" << v(0) << ", " << v(1) << ", " << v(2) << "]";
}

template<typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last)
            out << ", ";
    }
    out << "]";
    return out;
}

/** Interface for an environment, whose states can be represented as
    vectors of floats and whose actions can be represented as ints.
    Implementations of the Environment interface determine how actions
    influence sensations.  Note that this design assumes only one
    agent: it would be more accurate to name this interface
    EnvironmentAsPerceivedByOneParticularAgent. */
class Environment {
public:
  /** Provides access to the current sensation that the environment
      gives to the agent.
      \return The current sensation. */
  virtual const std::vector<float> &sensation() = 0;

  /** Allows an agent to affect its environment.
      \param action The action the agent wishes to apply.
      \return The immediate one-step reward caused by the action. */
  virtual float apply(std::vector<float> action) = 0;

  /** Determines whether the environment has reached a terminal state.
      \return true iff the task is episodic and the present episode
      has ended.  Nonepisodic tasks should simply always
      return false. */
  virtual bool terminal() = 0;

  /** Resets the internal state of the environment according to some
      initial state distribution.  Typically the user calls this only
      for episodic tasks that have reached terminal states, but this
      usage is not required. */
  virtual void reset() = 0;

  virtual ~Environment() {};

};

/** Interface for an agent.  Implementations of the Agent interface
    determine the choice of actions given previous sensations and
    rewards. */
class Agent {
public:
  /** Determines the first action that an agent takes in an
      environment.  This method implies that the environment is
      currently in an initial state.
      \param s The initial sensation from the environment.
      \return The action vector the agent wishes to take first. */
  virtual std::vector<float> first_action(const std::vector<float> &s) = 0;

  /** Determines the next action that an agent takes in an environment
      and gives feedback for the previous action.  This method may
      only be called if the last method called was first_action or
      next_action.
      \param r The one-step reward resulting from the previous action.
      \param s The current sensation from the environment.
      \return The action vector the agent wishes to take next. */
  virtual std::vector<float> next_action(float r, const std::vector<float> &s) = 0;

  /** Gives feedback for the last action taken.  This method may only
      be called if the last method called was first_action or
      next_action.  It implies that the task is episodic and has just
      terminated.  Note that terminal sensations (states) are not
      represented.
      \param r The one-step reward resulting from the previous action. */
  virtual void last_action(float r) = 0;

  virtual ~Agent() {};
};

#endif
