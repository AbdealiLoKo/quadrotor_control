import numpy

class Agent:
    def __init__(self, num_states, num_actions):
        self.state = None
        self.reward = None
        self.action = None

        self.old_state = None
        self.old_action = None

        self.stepsize = "harmonic"
        # harmonic = 1/n

        self.discounting_factor = 0.5

        self.Q = numpy.zeros([num_states, num_actions])
        self.num_action = numpy.zeros([num_actions,])

    def action_value(self, state, action):
        return self.Q[state, action]

    def policy(self, state):
        action = np.argwhere(
            self.Q[:,i_run] == self.Q[:,i_run].max()).flatten()
        if a_star.shape[0] > 1: # Incase there are more than 1 optimal action (very rare) give a warning
        


    def update_rule(self, next_state, next_action=None):
        if self.stepsize = "harmonic":
            stepsize = 1.0 / self.num_action[]
        Q[state, action] = Q[state, action] + \
            self.stepsize
