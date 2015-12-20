#include <ros/ros.h>
#include "std_msgs/String.h"

#include <rl_msgs/RLStateReward.h>
#include <rl_msgs/RLAction.h>
#include <rl_msgs/RLExperimentInfo.h>

#include <rl_common/core.hh>

#include <rl_env/HectorQuad.hh>

#include <getopt.h>
#include <stdlib.h>

static ros::Publisher out_env_desc;
static ros::Publisher out_env_sr;
static ros::Publisher out_seed;

Environment* environment;
int seed = 1;
std::string env_type = "";

void display_help() {
  std::cout << "\n env --env type [options]\n";
  std::cout << "\n Options:\n";
  std::cout << "--env type (Env types: hectorquad)\n";
  exit(-1);
}

void process_action(const rl_msgs::RLAction::ConstPtr &actionIn) {
  // Get action from agent and give back the next state
  rl_msgs::RLStateReward sr;
  sr.reward = environment->apply(actionIn->action);
  sr.state = environment->sensation();
  sr.terminal = environment->terminal();

  out_env_sr.publish(sr);
}

void process_episode(const rl_msgs::RLExperimentInfo::ConstPtr &infoIn) {
  // Process end-of-episode reward info. Mostly to start new episode.
  environment->reset();

  rl_msgs::RLStateReward sr;
  sr.reward = 0;
  sr.state = environment->sensation();
  sr.terminal = false;

  out_env_sr.publish(sr);
}

void init_env() {
  // Initialize the environment
  environment = NULL;

  if (env_type == "hectorquad"){
    environment = new HectorQuad();
  } else {
    std::cerr << "Invalid env type\n";
    display_help();
    exit(-1);
  }

  // Send first `state`
  rl_msgs::RLStateReward sr;
  sr.reward = 0;
  sr.state = environment->sensation();
  sr.terminal = false;

  out_env_sr.publish(sr);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "RLEnvironment");
  ros::NodeHandle node;

  // Parse options
  char ch;
  const char* optflags = "es";
  int option_index = 0;
  static struct option long_options[] = {
    {"env", 1, 0, 'e'},
    {"seed", 1, 0, 's'},
    {NULL, 0, 0, 0}
  };

  while(-1 != (ch = getopt_long_only(argc, argv, optflags, long_options, &option_index))) {
    switch(ch) {
    case 's':
      seed = std::atoi(optarg);
      std::cout << "Using seed: " << seed << "\n";
      break;

    case 'e':
      env_type = optarg;
      std::cout << "Using environment type: " << env_type << "\n";
      break;

    default:
      display_help();
      break;
    }
  }

  if (env_type == ""){
    env_type = "hectorquad";
    std::cout << "--env not given. Using HectorQuad\n";
  }

  std::cout << "RL ENV:  Initializing ROS ...\n";
  // Publishers
  out_env_sr = node.advertise<rl_msgs::RLStateReward>("rl_env/rl_state_reward",
                                                      1,
                                                      false);

  // Subscribers
  ros::TransportHints no_delay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber rl_action =  node.subscribe("rl_agent/rl_action",
                                              1,
                                              process_action,
                                              no_delay);
  ros::Subscriber rl_exp_info =  node.subscribe("rl_agent/rl_experiment_info",
                                                1,
                                                process_episode,
                                                no_delay);

  init_env();
  ROS_INFO("RL ENV: starting main loop");
  ros::spin();
  return 0;
}
