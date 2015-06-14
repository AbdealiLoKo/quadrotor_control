#include <ros/ros.h>

#include <rl_msgs/RLStateReward.h>
#include <rl_msgs/RLEnvDescription.h>
#include <rl_msgs/RLAction.h>
#include <rl_msgs/RLExperimentInfo.h>
#include <rl_msgs/RLEnvSeedExperience.h>

#include <ros/callback_queue.h>

#include <rl_common/core.hh>
#include <rl_common/Random.h>
#include <rl_common/ExperienceFile.hh>

#include <rl_agent/DiscretizationAgent.hh>
#include <rl_agent/QLearner.hh>
#include <rl_agent/SavedPolicy.hh>
#include <rl_agent/Sarsa.hh>

#include "std_msgs/String.h"

#include <getopt.h>
#include <stdlib.h>

#define NODE "RLAgent"

static ros::Publisher out_rl_action;
static ros::Publisher out_exp_info;

bool firstAction = true;
int seed = 1;

Agent* agent = NULL;
bool PRINTS = 0;

rl_msgs::RLExperimentInfo info;
char* agentType;

// default parameters
float _gamma = 0.99; // "gamma" is already declared in ros/ros.h
float epsilon = 0.1;
float alpha = 0.3;
float initialvalue = 0.0;
float lambda = 0;
int nstates = 0;
char *filename = NULL;
// possibly over-written by command line arguments


void displayHelp(){
  std::cout << "\n Call agent --agent type [options]\n";
  std::cout << "Agent types: qlearner sarsa savedpolicy\n";
  std::cout << "\n Options:\n";
  std::cout << "--seed value (integer seed for random number generator)\n";
  std::cout << "--gamma value (discount factor between 0 and 1)\n";
  std::cout << "--epsilon value (epsilon for epsilon-greedy exploration)\n";
  std::cout << "--alpha value (learning rate alpha)\n";
  std::cout << "--initialvalue value (initial q values)\n";
  std::cout << "--lamba value (lamba for eligibility traces)\n";
  std::cout << "--nstates value (optionally discretize domain into value # of states on each feature)\n";
  std::cout << "--filename file (file to load saved policy from for savedpolicy agent)\n";
  std::cout << "--prints (turn on debug printing of actions/rewards)\n";
  exit(-1);
}

void processState(const rl_msgs::RLStateReward::ConstPtr &stateIn){
  /** Process the state/reward message from the environment */

  if (agent == NULL){
    std::cout << "no agent yet" << endl;
    return;
  }

  rl_msgs::RLAction a;

  // first action
  if (firstAction){
    a.action = agent->first_action(stateIn->state);
    info.episode_reward = 0;
    info.number_actions = 1;
    if (PRINTS >= 2) std::cout << NODE << " Episode " << info.episode_number << ", First Action, reward: " << info.episode_reward << endl;
  } else {
    info.episode_reward += stateIn->reward;
    // if terminal, no action, but calculate reward sum
    if (stateIn->terminal){
      agent->last_action(stateIn->reward);
      std::cout << NODE << " Episode " << info.episode_number << ", Last Action (" << info.number_actions << "), reward: " << info.episode_reward << endl;
      // publish episode reward message
      out_exp_info.publish(info);
      info.episode_number++;
      info.episode_reward = 0;
      firstAction = true;
      return;
    } else {
      a.action = agent->next_action(stateIn->reward, stateIn->state);
      info.number_actions++;
      if (PRINTS >= 2) std::cout << NODE << " Episode " << info.episode_number << ", Action " << info.number_actions << ", reward: " << info.episode_reward << endl;
    }
  }
  firstAction = false;

  // publish agent's action
  out_rl_action.publish(a);
}


void processSeed(const rl_msgs::RLEnvSeedExperience::ConstPtr &seedIn){
  /** Process seeds for initializing model */

  if (agent == NULL){
    std::cout << NODE << " no agent yet" << endl;
    return;
  }

  if (PRINTS >= 2) std::cout << NODE << " Got a seed.";

  std::vector<experience> seeds;
  experience seed1;
  seed1.s = seedIn->from_state;
  seed1.next = seedIn->to_state;
  seed1.act = seedIn->action;
  seed1.reward = seedIn->reward;
  seed1.terminal = seedIn->terminal;
  seeds.push_back(seed1);

  agent->seedExp(seeds);

}


void processEnvDescription(const rl_msgs::RLEnvDescription::ConstPtr &envIn){
  /** Process the env description message from the environment */

  // initialize the agent based on some info from the environment descriptor
  Random rng(seed+1);
  agent = NULL;


  if (strcmp(agentType, "qlearner") == 0){
    std::cout << "Agent: QLearner" << endl;
    agent = new QLearner(envIn->num_actions, _gamma, initialvalue,
                         alpha, epsilon, rng);
  } else if (strcmp(agentType, "sarsa") == 0){
    std::cout << "Agent: Sarsa" << endl;
    agent = new Sarsa(envIn->num_actions, _gamma, initialvalue, alpha,
                      epsilon, lambda, rng);
  } else if (strcmp(agentType, "savedpolicy") == 0){
    std::cout << "Agent: Saved Policy" << endl;
    agent = new SavedPolicy(envIn->num_actions, filename);
  } else {
    std::cout << "Invalid Agent!" << endl;
    displayHelp();
    exit(-1);
  }

  Agent* a2 = agent;
  // not for model based when doing continuous model
  if (nstates > 0){
    int totalStates = powf(nstates,envIn->min_state_range.size());
    if (PRINTS >= 2) std::cout << "Discretize with " << nstates << ", total: " << totalStates << endl;
    agent = new DiscretizationAgent(nstates, a2,
                                    envIn->min_state_range,
                                    envIn->max_state_range, PRINTS > 0);
  }


  firstAction = true;
  info.episode_number = 0;
  info.episode_reward = 0;
}



int main(int argc, char *argv[]) {
  /** Main method to start up agent node. */
  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  // agent is required required
  if (argc < 3){
    std::cout << "--agent type  option is required" << endl;
    displayHelp();
    exit(-1);
  }

  // parse options to change these parameters
  agentType = argv[1];
  seed = std::atoi(argv[2]);

  // parse agent type first
  bool gotAgent = false;
  for (int i = 1; i < argc-1; i++){
    if (strcmp(argv[i], "--agent") == 0){
      gotAgent = true;
      agentType = argv[i+1];
    }
  }
  if (!gotAgent) {
    std::cout << "--agent type  option is required" << endl;
    displayHelp();
  }

  char ch;
  const char* optflags = "geailsqdwf";
  int option_index = 0;
  static struct option long_options[] = {
    {"gamma", 1, 0, 'g'},
    {"epsilon", 1, 0, 'e'},
    {"alpha", 1, 0, 'a'},
    {"initialvalue", 1, 0, 'i'},
    {"lambda", 1, 0, 'l'},
    {"seed", 1, 0, 's'},
    {"agent", 1, 0, 'q'},
    {"prints", 1, 0, 'd'},
    {"nstates", 1, 0, 'w'},
    {"filename", 1, 0, 'f'},
    {NULL, 0, 0, 0}
  };

  while(-1 != (ch = getopt_long_only(argc, argv, optflags, long_options, &option_index))) {
    switch(ch) {

    case 'g':
      _gamma = std::atof(optarg);
      std::cout << "gamma: " << _gamma << endl;
      break;

    case 'e':
      epsilon = std::atof(optarg);
      std::cout << "epsilon: " << epsilon << endl;
      break;

    case 'f':
      filename = optarg;
      std::cout << "policy filename: " <<  filename << endl;
      break;

    case 'a':
      {
        if (strcmp(agentType, "qlearner") == 0 || strcmp(agentType, "sarsa") == 0){
          alpha = std::atof(optarg);
          std::cout << "alpha: " << alpha << endl;
        } else {
          std::cout << "--alpha option is only valid for Q-Learning and Sarsa" << endl;
          exit(-1);
        }
        break;
      }

    case 'i':
      {
        if (strcmp(agentType, "qlearner") == 0 || strcmp(agentType, "sarsa") == 0){
          initialvalue = std::atof(optarg);
          std::cout << "initialvalue: " << initialvalue << endl;
        } else {
          std::cout << "--initialvalue option is only valid for Q-Learning and Sarsa" << endl;
          exit(-1);
        }
        break;
      }

    case 'l':
      {
        if (strcmp(agentType, "sarsa") == 0){
          lambda = std::atof(optarg);
          std::cout << "lambda: " << lambda << endl;
        } else {
          std::cout << "--lambda option is invalid for this agent: " << agentType << endl;
          exit(-1);
        }
        break;
      }


    case 's':
      seed = std::atoi(optarg);
      std::cout << "seed: " << seed << endl;
      break;

    case 'q':
      // already processed this one
      std::cout << "agent: " << agentType << endl;
      break;

    case 'd':
      PRINTS = std::atoi(optarg);
      break;

    case 'w':
      nstates = std::atoi(optarg);
      std::cout << "nstates for discretization: " << nstates << endl;
      break;

    case 'h':
    case '?':
    case 0:
    default:
      displayHelp();
      break;
    }
  }

  int qDepth = 1;

  std::cout << NODE << " Setting up Publishers ...\n";
  out_rl_action = node.advertise<rl_msgs::RLAction>("rl_agent/rl_action",qDepth, false);
  out_exp_info = node.advertise<rl_msgs::RLExperimentInfo>("rl_agent/rl_experiment_info",qDepth, false);

  std::cout << NODE << " Setting up Subscribers ...\n";
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber rl_description =  node.subscribe("rl_env/rl_env_description", qDepth, processEnvDescription, noDelay);
  ros::Subscriber rl_state =  node.subscribe("rl_env/rl_state_reward", qDepth, processState, noDelay);
  ros::Subscriber rl_seed =  node.subscribe("rl_env/rl_seed", 20, processSeed, noDelay);

  ROS_INFO(NODE ": starting main loop");

  ros::spin();                          // handle incoming data

  if(agent != NULL) {
    if(filename != NULL) {
      agent->savePolicy(filename);
    }
  }

  return 0;
}
