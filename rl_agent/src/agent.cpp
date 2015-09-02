// C ++
#include <getopt.h>
// ROS
#include <ros/ros.h>
// Messages
#include <rl_msgs/RLStateReward.h>
#include <rl_msgs/RLEnvDescription.h>
#include <rl_msgs/RLAction.h>
#include <rl_msgs/RLExperimentInfo.h>
#include <rl_msgs/RLEnvSeedExperience.h>
// RL ROS files
#include <rl_common/core.hh>
#include <rl_common/Random.h>
#include <rl_common/ExperienceFile.hh>
// Agents
#include <rl_agent/NullAgent.hh>
#include <rl_agent/DiscretizationAgent.hh>
#include <rl_agent/Pegasus.hh>
#include <rl_agent/QLearner.hh>
#include <rl_agent/SavedPolicy.hh>
#include <rl_agent/Sarsa.hh>

#define NODE "RLAgent"

static ros::Publisher out_rl_action;
static ros::Publisher out_exp_info;

bool firstAction = true;
int seed = 1;

Agent* agent = NULL;
bool PRINTS = 0;
int counttotal = 0;

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
  std::cout << std::endl << " Call agent --agent type [options]" << std::endl;
  std::cout << "Agent types: qlearner sarsa savedpolicy pegasus" << std::endl;
  std::cout << std::endl << " Options:" << std::endl;
  std::cout << "--seed value (integer seed for random number generator)"
            << std::endl;
  std::cout << "--gamma value (discount factor between 0 and 1)" << std::endl;
  std::cout << "--epsilon value (epsilon for epsilon-greedy exploration)"
            << std::endl;
  std::cout << "--alpha value (learning rate alpha)" << std::endl;
  std::cout << "--initialvalue value (initial q values)" << std::endl;
  std::cout << "--lamba value (lamba for eligibility traces)" << std::endl;
  std::cout << "--nstates value (optionally discretize domain into value # of "
               "states on each feature)" << std::endl;
  std::cout << "--filename file (file to load saved policy from for "
               "savedpolicy agent)" << std::endl;
  std::cout << "--prints (turn on debug printing of actions/rewards)"
            << std::endl;
  exit(-1);
}

void processState(const rl_msgs::RLStateReward::ConstPtr &stateIn){
  /** Process the state/reward message from the environment */

  if (agent == NULL){
    std::cout << "no agent yet" << std::endl;
    return;
  }

  rl_msgs::RLAction a;

  // first action
  if (firstAction){
    a.action = agent->first_action(stateIn->state);
    info.episode_reward = 0;
    info.number_actions = 1;
    if (PRINTS >= 2)
      std::cout << NODE << " Episode " << info.episode_number << ", First "
                   "Action, reward: " << info.episode_reward << std::endl;
  } else {
    info.episode_reward += stateIn->reward;
    // if terminal, no action, but calculate reward sum
    if (stateIn->terminal || counttotal>1000){
      counttotal=0;
      agent->last_action(stateIn->reward);
      std::cout << NODE << " Episode " << info.episode_number << ", Last "
                   "Action (" << info.number_actions << "), reward: "
                   << info.episode_reward << std::endl;
      // publish episode reward message
      out_exp_info.publish(info);
      info.episode_number++;
      firstAction = true;
      return;
    } else {  
      a.action = agent->next_action(stateIn->reward, stateIn->state);
      info.number_actions++;
      if (PRINTS >= 2)
        std::cout << NODE << " Episode " << info.episode_number << ", Action "
                  << info.number_actions << ", reward: " << info.episode_reward
                  << std::endl;
    }
  }
  firstAction = false;
  counttotal++;
  // publish agent's action
  out_rl_action.publish(a);
}


void processSeed(const rl_msgs::RLEnvSeedExperience::ConstPtr &seedIn){
  /** Process seeds for initializing model */

  if (agent == NULL){
    std::cout << NODE << " no agent yet" << std::endl;
    return;
  }

  if (PRINTS >= 2)
    std::cout << NODE << " Got a seed.";

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


  if (strcmp(agentType, "null") == 0){
    std::cout << "Agent: NullAgent" << std::endl;
    agent = new NullAgent();
  } else if (strcmp(agentType, "qlearner") == 0){
    std::cout << "Agent: QLearner" << std::endl;
    agent = new QLearner(envIn->num_actions, _gamma, initialvalue,
                         alpha, epsilon, rng);
  } else if (strcmp(agentType, "sarsa") == 0){
    std::cout << "Agent: Sarsa" << std::endl;
    agent = new Sarsa(envIn->num_actions, _gamma, initialvalue, alpha,
                      epsilon, lambda, rng);
  } else if (strcmp(agentType, "savedpolicy") == 0){
    std::cout << "Agent: Saved Policy" << std::endl;
    agent = new SavedPolicy(envIn->num_actions, filename);
  } else if (strcmp(agentType, "pegasus") == 0){
    std::cout << "Agent: Pegasus" << std::endl;
    agent = new Pegasus(2, 1, 0.01, 0.9, rng);
  } else {
    std::cout << "Invalid Agent!" << std::endl;
    displayHelp();
    exit(-1);
  }

  Agent* a2 = agent;
  // not for model based when doing continuous model
  if (nstates > 0){
    int totalStates = pow(nstates,envIn->min_state_range.size());
    if (PRINTS >= 2)
      std::cout << "Discretize with " << nstates << ", total: " << totalStates
                << std::endl;
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
    std::cout << "--agent type  option is required" << std::endl;
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
    std::cout << "--agent type  option is required" << std::endl;
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
      std::cout << "gamma: " << _gamma << std::endl;
      break;

    case 'e':
      epsilon = std::atof(optarg);
      std::cout << "epsilon: " << epsilon << std::endl;
      break;

    case 'f':
      filename = optarg;
      std::cout << "policy filename: " <<  filename << std::endl;
      break;

    case 'a':
      {
        if (strcmp(agentType, "qlearner") == 0 || strcmp(agentType, "sarsa") == 0){
          alpha = std::atof(optarg);
          std::cout << "alpha: " << alpha << std::endl;
        } else {
          std::cout << "--alpha option is only valid for Q-Learning and Sarsa"
                    << std::endl;
          exit(-1);
        }
        break;
      }

    case 'i':
      {
        if (strcmp(agentType, "qlearner") == 0 || strcmp(agentType, "sarsa") == 0){
          initialvalue = std::atof(optarg);
          std::cout << "initialvalue: " << initialvalue << std::endl;
        } else {
          std::cout << "--initialvalue option is only valid for Q-Learning "
                       "and Sarsa" << std::endl;
          exit(-1);
        }
        break;
      }

    case 'l':
      {
        if (strcmp(agentType, "sarsa") == 0){
          lambda = std::atof(optarg);
          std::cout << "lambda: " << lambda << std::endl;
        } else {
          std::cout << "--lambda option is invalid for this agent: "
                    << agentType << std::endl;
          exit(-1);
        }
        break;
      }


    case 's':
      seed = std::atoi(optarg);
      std::cout << "seed: " << seed << std::endl;
      break;

    case 'q':
      // already processed this one
      std::cout << "agent: " << agentType << std::endl;
      break;

    case 'd':
      PRINTS = std::atoi(optarg);
      break;

    case 'w':
      nstates = std::atoi(optarg);
      std::cout << "nstates for discretization: " << nstates << std::endl;
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

  std::cout << NODE << " Setting up Publishers ..." << std::endl;
  out_rl_action = node.advertise<rl_msgs::RLAction>("rl_agent/rl_action",
                                                    qDepth,
                                                    false);
  out_exp_info = node.advertise<rl_msgs::RLExperimentInfo>(
    "rl_agent/rl_experiment_info",
    qDepth,
    false);

  std::cout << NODE << " Setting up Subscribers ..." << std::endl;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber rl_description =  node.subscribe("rl_env/rl_env_description",
                                                   qDepth,
                                                   processEnvDescription,
                                                   noDelay);
  ros::Subscriber rl_state =  node.subscribe("rl_env/rl_state_reward",
                                             qDepth,
                                             processState,
                                             noDelay);
  ros::Subscriber rl_seed =  node.subscribe("rl_env/rl_seed",
                                            20,
                                            processSeed,
                                            noDelay);

  ROS_INFO(NODE ": starting main loop");

  ros::spin(); // handle incoming data

  if(agent != NULL) {
    if(filename != NULL) {
      agent->savePolicy(filename);
    }
  }

  return 0;
}
