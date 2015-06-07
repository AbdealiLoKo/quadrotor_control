#!/usr/bin/env python

import random

import rospy

from rl_py.msg import (RLEnvDescription, RLStateReward, RLEnvSeedExperience,
                       RLAction, RLExperimentInfo)
from rl_agent.cli_args import parser
from rl_agent.Sarsa import Sarsa
from rl_agent.Agent import Agent

NODE = "RLAgent"

out_rl_action = None
out_exp_info = None

first_action = True

agent = None
args = None

info = RLExperimentInfo()
rng = None
# int model = C45TREE;
# int explore = GREEDY;
# int modelcombo = BEST;
# int planner = PAR_ETUCT_ACTUAL;


# Process the state/reward message from the environment
def process_state(state_in):
    global info, first_action

    if agent == None:
        print("No agent yet")
        return

    a = RLAction()

    # First action
    if first_action:
        a.action = agent.first_action(state_in.state);
        info.episode_reward = 0;
        info.number_actions = 1;
    else:
        info.episode_reward += state_in.reward;
        # if terminal, no action, but calculate reward sum
        if state_in.terminal:
            agent.last_action(state_in.reward);
            print("Episode", info.episode_number, "reward:", info.episode_reward)
            # publish episode reward message
            out_exp_info.publish(info);

            info.episode_number += 1
            info.episode_reward = 0;
            first_action = True;
            return
        else:
            a.action = agent.next_action(state_in.reward, state_in.state)
            info.number_actions += 1

    first_action = false;

    # publish agent's action
    out_rl_action.publish(a);


# Process seeds for initializing model
def processSeed(seed_in):
    if agent == None:
        print("No agent yet")
        return

    seeds = []
    seed1 = Experience()
    seed1.s = seed_in.from_state;
    seed1.next = seed_in.to_state;
    seed1.act = seed_in.action;
    seed1.reward = seed_in.reward;
    seed1.terminal = seed_in.terminal;
    seeds.append(seed1);

    agent.seed_exp(seeds);

# Process the env description message from the environment
def process_env_description(env_in):
    global agent, first_action, info
    # Initialize the agent based on some info from the environment descriptor
    agent = None;

    if args.type == "qlearner":
        print("Agent: QLearner")
        agent = QLearner(num_actions=env_in.num_actions,
                         gamma=args.gamma,
                         initialvalue=args.initialvalue,
                         alpha=args.alpha,
                         epsilon=args.epsilon,
                         rng=rng)
    elif ards.type == "sarsa":
        print("Agent: Sarsa")
        agent = Sarsa(num_actions=env_in.num_actions,
                      gamma=args.gamma,
                      initialvalue=args.initialvalue,
                      alpha=args.alpha,
                      epsilon=args.epsilon,
                      _lambda=args._lambda,
                      rng=rng)
    else:
        print("Invalid Agent!")
        sys.exit(-1)

  #   Agent* a2 = agent;
  # // not for model based when doing continuous model
  # if (nstates > 0 && (model != M5ALLMULTI || strcmp(agentType, "qlearner") == 0)){
  #   int totalStates = powf(nstates,envIn->min_state_range.size());
  #   if (PRINTS) cout << "Discretize with " << nstates << ", total: " << totalStates << endl;
  #   agent = new DiscretizationAgent(nstates, a2,
  #                                   envIn->min_state_range,
  #                                   envIn->max_state_range, PRINTS);
  # }

    first_action = True;
    info.episode_number = 0;
    info.episode_reward = 0;

if __name__ == "__main__":
    args = parser.parse_args()
    rng = random.Random()
    queue_depth = 1

    # Change to dictionarized exploration
    if args.exploration == "unknown":
        args.exploration = "EXPLORE_UNKNOWN"
    elif args.exploration == "greedy":
        args.exploration = "GREEDY"
    elif args.exploration == "epsilongreedy":
        args.exploration = "EPSILONGREEDY"
    elif args.exploration == "unvisitedstates":
        args.exploration = "UNVISITED_BONUS"
    elif args.exploration == "unvisitedactions":
        args.exploration = "UNVISITED_ACT_BONUS"
    elif args.exploration == "variancenovelty":
        args.exploration = "DIFF_AND_NOVEL_BONUS"

    if args.type != "sarsa":
        raise NotImplementedError()

    # Make a node
    rospy.loginfo("Making a node ...")
    rospy.init_node(NODE, anonymous=True)

    # Publishers
    rospy.loginfo("Making publishers ...")
    out_rl_action = rospy.Publisher("rl_agent/rl_action", RLAction, queue_size=queue_depth, latch=False)
    out_exp_info = rospy.Publisher("rl_agent/rl_experiment_info", RLExperimentInfo, queue_size=queue_depth, latch=False)

    # Subscribers
    rospy.loginfo("Making subscribers ...")
    rl_description = rospy.Subscriber("rl_env/rl_env_description", queue_depth, process_env_description, tcp_nodelay=True)
    rl_state = rospy.Subscriber("rl_env/rl_state_reward", queue_depth, process_state, tcp_nodelay=True)
    rl_seed = rospy.Subscriber("rl_env/rl_seed", queue_depth, process_seed, tcp_nodelay=True)

    # Setup RL World
    rospy.loginfo("Setting up RL Agent ...")
    rng.seed(1+args.seed)
    init_environment(rng)

    rospy.loginfo(NODE + ": starting main loop")
    rospy.spin()
