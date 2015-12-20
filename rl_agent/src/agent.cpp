#include <getopt.h>
#include <ros/ros.h>

#include <rl_common/core.hh>

// Messages
#include <rl_common/RLStateReward.h>
#include <rl_common/RLAction.h>
#include <rl_common/RLExperimentInfo.h>

// Agents
#include <rl_agent/Pegasus.hh>


static ros::Publisher out_rl_action;
static ros::Publisher out_exp_info;


const int MAX_STEPS = 10000;
Agent* agent = NULL;
int seed = 1;

rl_common::RLExperimentInfo info;
std::string agent_type = "";

void display_help(){
  std::cout << "\n agent --agent type [options]\n";
  std::cout << "\n Options:\n";
  std::cout << "--agent type (Agent types: pegasus)\n";
  exit(-1);
}

void init_agent() {
  agent = NULL;

  if (agent_type == "pegasus"){
    std::cout << "Agent: Pegasus" << std::endl;
    // For now, we arent using these args. Theyre reset in the constructor
    agent = new Pegasus(4, // Num state
                        2 // Num action
                        );
  } else {
    std::cout << "Invalid Agent!" << std::endl;
    display_help();
  }

  info.number_actions = 0;
}

void process_state(const rl_common::RLStateReward::ConstPtr &state_in){
  /** Process the state/reward message from the environment */

  if (agent == NULL) {
    init_agent();
  }

  rl_common::RLAction msg;

  if (info.number_actions == 0) {
    msg.action = agent->first_action(state_in->state);
    info.episode_reward = 0;
    info.number_actions += 1;

  } else if (state_in->terminal || info.number_actions > MAX_STEPS) {
    info.episode_reward += state_in->reward;
    info.episode_number += 1;
    agent->last_action(state_in->reward);
    out_exp_info.publish(info); // Publish end of episode message

    // std::cout << "RL AGENT: Episode " << info.episode_number
    //           << ", #Actions " << info.number_actions
    //           << ", Episode Reward: " << info.episode_reward << "\n";

    info.number_actions = 0;
    info.episode_reward = 0;
    return;

  } else {
    info.episode_reward += state_in->reward;
    info.number_actions += 1;
    msg.action = agent->next_action(state_in->reward, state_in->state);
  }

  out_rl_action.publish(msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "RLAgent");
  ros::NodeHandle node;

  char ch;
  const char* optflags = "as";
  int option_index = 0;
  static struct option long_options[] = {
    {"seed", 1, 0, 's'},
    {"agent", 1, 0, 'a'},
    {NULL, 0, 0, 0}
  };

  while(-1 != (ch = getopt_long_only(argc, argv, optflags, long_options, &option_index))) {
    switch(ch) {

    case 's':
      seed = std::atoi(optarg);
      std::cout << "Using seed: " << seed << "\n";
      break;

    case 'a':
      agent_type = optarg;
      std::cout << "Using agent: " << agent_type << "\n";
      break;

    default:
      display_help();
      break;
    }
  }

  if (agent_type == "") {
    display_help();
  }

  std::cout << "RL AGENT:  Initializing ROS ...\n";
  // Publishers
  out_rl_action = node.advertise<rl_common::RLAction>("rl_agent/rl_action",
                                                    1,
                                                    false);
  out_exp_info = node.advertise<rl_common::RLExperimentInfo>(
    "rl_agent/rl_experiment_info",
    1,
    false);

  // Subscribers
  ros::TransportHints no_delay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber rl_state =  node.subscribe("rl_env/rl_state_reward",
                                             1,
                                             process_state,
                                             no_delay);

  ROS_INFO("RL AGENT: starting main loop");
  ros::spin(); // handle incoming data

  return 0;
}
