#!/usr/bin/env python

import sys
import os
import random

import rospy

from rl_env.Environment import Environment, Experience
from rl_env.cli_args import parser
from rl_py.msg import RLEnvDescription, RLStateReward, RLEnvSeedExperience
from rl_env.Taxi import Taxi

NODE = "RLEnvironment"

out_env_desc = None
out_env_sr = None
out_seed = None

env = None
args = None

# Process action from the agent
def process_action(actionIn):
    sr = RLStateReward()

    # process action from the agent, affecting the environment
    sr.reward = env.apply(actionIn.action)
    sr.state = env.sensation()
    sr.terminal = env.terminal()

    # publish the state-reward message
    if args.debug:
       print("Got action", actionIn.action, "at state:", sr.state[0],
            ",", sr.state[1], ", reward:", sr.reward)

    out_env_sr.publish(sr)

# Process end-of-episode reward info. Mostly to start new episode.
def process_episode_info(infoIn):
    # Start new episode if terminal
    if args.debug:
        print("Episode", infoIn.episode_number, "terminated with reward:",
              infoIn.episode_reward, ", start new episode")

    env.reset()

    sr = RLStateReward
    sr.reward = 0
    sr.state = env.sensation()
    sr.terminal = false

    out_env_sr.publish(sr)

# Init the environment, publish a description.
def init_environment(rng):
    global env
    # init the environment
    env = None
    desc = RLEnvDescription()
    stochastic = not args.deterministic

    if args.type == "taxi":
        desc.title = "Environment: Taxi\n"
        env = Taxi(rng, stochastic=stochastic)
    elif args.type == "cartpole":
        desc.title = "Environment: Cart Pole\n"
        env = CartPole(rng, stochastic)
    elif args.type == "mcar":
        desc.title = "Environment: Mountain Car\n"
        env = MountainCar(rng, stochastic, false, delay)
    elif args.type == "lightword":
        desc.title = "Environment: Light World\n"
        env = LightWorld(rng, stochastic, 4)
    elif args.type == "tworooms":
        desc.title = "Environment: TwoRooms\n"
        env = TwoRooms(rng, stochastic, true, arg.delay, false)
    elif args.type == "car2to7":
        desc.title = "Environment: Car Velocity 2 to 7 m/s\n"
        env = RobotCarVel(rng, false, true, false, args.lag)
    elif args.type == "car7to2":
        desc.title = "Environment: Car Velocity 7 to 2 m/s\n"
        env = RobotCarVel(rng, false, false, false, args.lag)
    elif args.type == "carrandom":
        desc.title = "Environment: Car Velocity Random Velocities\n"
        env = RobotCarVel(rng, true, false, false, args.lag)
    elif args.type == "fourrooms":
        desc.title = "Environment: FourRooms\n"
        env = FourRooms(rng, stochastic, true, false)
    elif arg.type == "energy":
        desc.title = "Environment: EnergyRooms\n"
        env = EnergyRooms(rng, stochastic, true, false)
    elif arg.type == "fuelworld":
        desc.title = "Environment: FuelWorld\n"
        env = FuelRooms(rng, args.highvar, stochastic)
    elif arg.type == "stocks":
        desc.title = "Environment: Stocks\n"
        env = Stocks(rng, stochastic, args.nsectors, args.nstocks)

    # Add some more meta data
    desc.num_actions = env.get_num_actions()
    desc.episodic = env.is_episodic()

    _min, _max = env.get_min_max_features()
    desc.num_states = len(_min)
    desc.min_state_range = _min
    desc.max_state_range = _max

    desc.stochastic = stochastic
    _min, _max = env.get_min_max_reward()
    desc.max_reward = _max
    desc.reward_range = _max - _min

    rospy.loginfo(desc.title)

    # Publish environment description
    out_env_desc.publish(desc)

    rospy.sleep(1)

    # Send initial seed experiences
    initial_experiences = env.get_seedings()
    for exp in initial_experiences:
        experience = RLEnvSeedExperience()
        experience.from_state = exp.s
        experience.to_state   = exp.next
        experience.action     = exp.act
        experience.reward     = exp.reward
        experience.terminal   = exp.terminal
        out_seed.publish(experience)

    # Now send first state message
    sr = RLStateReward()
    sr.terminal = False
    sr.reward = 0
    sr.state = env.sensation()
    out_env_sr.publish(sr)


if __name__ == "__main__":
    args = parser.parse_args()
    rng = random.Random()
    queue_depth = 1

    # Make a node
    rospy.loginfo("Making a node ...")
    rospy.init_node(NODE, anonymous=True)

    # Publishers
    rospy.loginfo("Making publishers ...")
    out_env_desc = rospy.Publisher("rl_env/rl_env_description", RLEnvDescription, queue_size=queue_depth, latch=True)
    out_env_sr = rospy.Publisher("rl_env/rl_state_reward", RLStateReward, queue_size=queue_depth, latch=False)
    out_seed = rospy.Publisher("rl_env/rl_seed", RLEnvSeedExperience, queue_size=queue_depth, latch=False)

    # Subscribers
    rospy.loginfo("Making subscribers ...")
    rl_action = rospy.Subscriber("rl_agent/rl_action", queue_depth, process_action, tcp_nodelay=True)
    rl_exp_info = rospy.Subscriber("rl_agent/rl_experiment_info", queue_depth, process_episode_info, tcp_nodelay=True)

    # Setup RL World
    rospy.loginfo("Setting up RL World ...")
    rng.seed(1+args.seed)
    init_environment(rng)

    rospy.loginfo(NODE + ": starting main loop")
    rospy.spin()
