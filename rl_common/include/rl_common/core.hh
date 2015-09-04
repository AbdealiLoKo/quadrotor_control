#ifndef _RLCORE_H_
#define _RLCORE_H_

#include "Random.h"
#include <vector>

/** Experience <s,a,s',r> struct */
struct experience {
  std::vector<float> s;
  int act;
  float reward;
  std::vector<float> next;
  bool terminal;
};

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
  virtual float apply(int action) = 0;

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

  /** Returns the number of actions available in this environment.
      \return The number of actions available */
  virtual int getNumActions() = 0;

  /** Gets the minimum and maximum of the features in the environment.
   */
  virtual void getMinMaxFeatures(std::vector<float> *minFeat,
                                 std::vector<float> *maxFeat) = 0;

  /** Gets the minimum and maximum one-step reward in the domain. */
  virtual void getMinMaxReward(float *minR, float *maxR) = 0;

  /** Returns if the domain is episodic (true by default). */
  virtual bool isEpisodic(){ return true; };

  /** Get seeding experiences for agent. */
  virtual std::vector<experience> getSeedings()
  {
    std::vector<experience> e;
    return e;
  } ;

  /** Set the current state for testing purposes. */
  virtual void setSensation(std::vector<float> s){};

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
      \return The action the agent wishes to take first. */
  virtual int first_action(const std::vector<float> &s) = 0;

  /** Determines the next action that an agent takes in an environment
      and gives feedback for the previous action.  This method may
      only be called if the last method called was first_action or
      next_action.
      \param r The one-step reward resulting from the previous action.
      \param s The current sensation from the environment.
      \return The action the agent wishes to take next. */
  virtual int next_action(float r, const std::vector<float> &s) = 0;

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
