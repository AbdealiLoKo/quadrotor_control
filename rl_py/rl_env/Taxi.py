"""
Defines the taxi domain, from:
Dietterich, "The MAXQ method for hierarchical reinforcement learning," ICML 1998.
"""
from __future__ import print_function
import random

from rl_env.Environment import Environment
from rl_env.Experience import Experience
from rl_env.Gridworld import Gridworld

class Taxi(Environment):
    def __init__(self, rand, width=None, height=None, gridworld=None, stochastic=False):
        """
        Creates a deterministic Taxi domain.
        :param rand:       Random number generator (used solely for random
                           initial states if stochastic is False).
        :param width:      Width of grid - not used if gridworld is given.
        :param height:     Height of grid - not used if gridworld is given.
        :param gridworld:  The map to use.
        :param stochastic: Whether to use nondeterministic actions and
                           fickle passenger.
        """
        if gridworld != None:
            self.grid = gridworld
        else:
            self.grid = Taxi.create_default_map()
        self.landmarks = Taxi.default_landmarks
        self.rng = rand
        self.s = {"ns": 0.0, "ew": 0.0, "pass": 0.0, "dest": 0.0}
        self.noisy = stochastic
        if stochastic != None:
            self.randomize_landmarks_to_corners();
        self.reset()

    def sensation(self):
        # print(self.landmarks[self.s['dest']], " - ", (self.s['ns'], self.s['ew']))
        return [self.s['ns'], self.s['ew'], self.s['pass'], self.s['dest']]

    def apply(self, action):
        if self.noisy:
            effect = self.add_noise(action)
        else:
            effect = action

        if effect == Taxi.NORTH:
            if not self.grid.wall(self.s['ns'], self.s['ew'], effect):
                self.s['ns'] += 1
                self.apply_fickle_passengers();
            return -1
        elif effect == Taxi.SOUTH:
            if not self.grid.wall(self.s['ns'], self.s['ew'], effect):
                self.s['ns'] -= 1
                self.apply_fickle_passengers();
            return -1
        elif effect == Taxi.EAST:
            if not self.grid.wall(self.s['ns'], self.s['ew'], effect):
                self.s['ew'] += 1
                self.apply_fickle_passengers();
            return -1
        elif effect == Taxi.WEST:
            if not self.grid.wall(self.s['ns'], self.s['ew'], effect):
                self.s['ew'] -= 1
                self.apply_fickle_passengers();
            return -1
        elif effect == Taxi.PICKUP:
            if self.s['pass'] < len(self.landmarks) and \
                (self.s['ns'], self.s['ew']) == self.landmarks[self.s['pass']]:
                self.s['pass'] = len(self.landmarks)
                self.fickle = self.noisy
                return -1
            else:
                return -10
        elif effect == Taxi.PUTDOWN:
            if self.s['pass'] == len(self.landmarks) and \
                (self.s['ns'], self.s['ew']) == self.landmarks[self.s['dest']]:
                self.s['pass'] = self.s['dest']
                return 20
            else:
                return -10
        else:
            rospy.loginfo("Unreachable point reached in Taxi::apply!!!")
            return 0

    def terminal(self):
        return self.s['pass'] == self.s['dest']

    def reset(self):
        self.s['ns'] = self.rng.randint(1, self.grid.height) - 1
        self.s['ew'] = self.rng.randint(1, self.grid.width) - 1
        self.s['pass'] = self.rng.randint(1, len(self.landmarks)) - 1
        self.s['dest'] = self.s['pass']
        while self.s['pass'] == self.s['dest']:
            self.s['dest'] = self.rng.randint(1, len(self.landmarks)) - 1
        self.fickle = False

    def get_num_actions(self):
        return 6

    def get_min_max_features(self):
        min_feat = {"ns" : 0.0, "ew": 0.0, "pass": 0.0, "dest": 0.0}
        max_feat = {"ns" : 4.0, "ew": 4.0, "pass": 4.0, "dest": 4.0}

        min_feat = [min_feat['ns'], min_feat['ew'], min_feat['pass'], min_feat['dest']]
        max_feat = [max_feat['ns'], max_feat['ew'], max_feat['pass'], max_feat['dest']]

        return min_feat, max_feat

    def get_min_max_reward(self):
        min_reward = -10.0
        max_reward = 20.0
        return min_reward, max_reward

    def get_seedings(self):
        seeds = []

        if True: # Remove to use seeds
            return seeds

        # Single seed for each of 4 drop off and pickup cases
        for i in xrange(4):
            # drop off
            seeds.append(get_exp(self.landmarks[i].first, self.landmarks[i].second, 4, i, PUTDOWN))
            # pick up
            seeds.append(get_exp(self.landmarks[i].first, self.landmarks[i].second, i, self.rng.randint(0, 3), PUTDOWN))
        self.reset()
        return seeds

    def set_sensation(self, new_sensation):
        if len(self.s) != len(new_sensation) and \
            set(self.s.viewkeys()) == set(new_sensation.viewkeys()):
            rospy.logerr("Error in sensation sizes")
        self.s = new_sensation

    def get_exp(s0, s1, s2, s3, a):
        """
        Get an example experience for the given state-action.
        """
        exp = Experience()

        exp.s = [0] * 4
        exp.next = [0] * 4

        self.s['ns'] = s0
        self.s['ew'] = s1
        self.s['pass'] = s2
        self.s['dest'] = s3

        exp.act = a;
        exp.s = self.sensation()
        exp.reward = self.apply(exp.act)

        exp.terminal = self.terminal()
        exp.next = self.sensation()

        return exp

    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3
    PICKUP = 4
    PUTDOWN = 5

    default_landmarks = [ (4.0, 0.0), (0.0, 3.0), (4.0, 4.0), (0.0, 0.0) ]

    @staticmethod
    def create_default_map():
        """
        Creates the default gridworld map
        """
        nsv = [ [False] * 4 for i in xrange(5)]
        ewv = [ [False] * 4 for i in xrange(5)]
        ewv[0][0] = True
        ewv[0][2] = True
        ewv[1][0] = True
        ewv[1][2] = True
        ewv[3][1] = True
        ewv[4][1] = True
        grid = Gridworld(5, 5, northsouth=nsv, eastwest=ewv);
        return grid

    def add_noise(self, action):
        """
        Corrupts the movement action

        :param action: The intended action.
        :return:       The action actually executed.
        """
        if action == Taxi.NORTH or action == Taxi.SOUTH:
            if self.rng.random() > 0.8:
                return action
            else:
                if self.rng.random() > 0.5:
                    return Taxi.EAST
                else:
                    return Taxi.WEST
        elif action == Taxi.EAST or action == Taxi.WEST:
            if self.rng.random() > 0.8:
                return action
            else:
                if self.rng.random() > 0.5:
                    return Taxi.NORTH
                else:
                    return Taxi.SOUTH
        else:
            return action

    def apply_fickle_passengers(self):
        """
        If the domain is noisy and the taxi has just taken its first
        step since picking up the passenger, then potentially change the
        destination.
        """
        if self.fickle:
            self.fickle = False
            if self.rng.random() > 0.3:
                self.s['dest'] += self.rng.randint(1, len(self.landmarks) - 1)
                self.s['dest'] = self.s['dest'] % len(self.landmarks)

    def randomize_landmarks(self):
        """
        Randomly assigns the four landmarks.
        """
        indices = []
        n = self.grid.height * self.grid.width
        for i in xrange(len(self.landmarks)):
            index = None
            duplicate = True
            while duplicate:
                index = self.rng.randint(1, n) - 1;
                duplicate = index in indices
            indices.append(index)
        for i in xrange(len(indices)):
            self.landmarks[i] = (indices[i] / self.grid.width,
                                 indices[i] % self.grid.width)

    def randomize_landmarks_to_corners(self):
        """
        Randomly assigns the four landmarks to any four distinct
        positions in the world.
        """
        for i in xrange(len(self.landmarks)):
            ns = self.rng.randint(0, 1)
            ew = self.rng.randint(0, 1)
            if i / 2 == 1:
                ns = self.grid.height - ns - 1
            if i % 2 == 1:
                ew = self.grid.width - ew - 1
            self.landmarks[i] = (ns, ew)
