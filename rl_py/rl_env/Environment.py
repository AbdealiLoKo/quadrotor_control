#!/usr/bin/env python


class Environment:
    """
    Interface for an environment, whose states can be represented as lists
    and whose actions can be represented as ints. Implementations of the
    Environment interface determine how actions influence sensations.
    Note that this design assumes only one agent: it would be more accurate to
    name this interface EnvironmentAsPerceivedByOneParticularAgent
    ... but that's too long
    """
    def sensation(self):
        """
        Provides access to the current sensation that the environment
        gives to the agent.

        :return: The current sensation.
        """
        raise NotImplementedError()

    def apply(self, action):
        """
        Allows an agent to affect its environment by applying an action.

        :param action: The action the agent wishes to apply.
        :return:       The immediate one-step reward due to the action.
        """
        raise NotImplementedError()

    def terminal(self):
        """
        Determines whether the environment has reached a terminal state.

        :return: true iff the task is episodic and the present episode
                 has ended.  Nonepisodic tasks should simply always
                 return false.
        """
        raise NotImplementedError()

    def reset(self):
        """
        Resets the internal state of the environment according to some
        initial state distribution. Typically the user calls this only
        for episodic tasks that have reached terminal states, but this
        usage is not required.
        """
        raise NotImplementedError()

    def get_num_actions(self):
        """
        Returns the number of actions available in this environment.

        :return: The number of actions available
        """
        raise NotImplementedError()

    def get_min_max_features(self):
        """
        Gets the minimum and maximum of the features in the environment.

        :return: tuple containing the minimum and maximum features.
        """
        raise NotImplementedError()

    def get_min_max_reward(self):
        """
        Gets the minimum and maximum one-step reward in the domain.

        :return: Tuple containing the minimum and maximum reward.
        """

    def is_episodic(self):
        """
        Tells whether the domain is episodic or not. It is true by default.

        :return: True if the domain is episodic.
        """
        return True

    def get_seedings(self):
        """
        Get seeding experiences for agent.

        :return: List containing the seeding experiences.
        """

    def set_sensation(self, new_sensation):
        """
        Set the current state for testing purposes.

        """
        pass

if __name__ == "__main__":
    print "This file cannt be executed on it's own"