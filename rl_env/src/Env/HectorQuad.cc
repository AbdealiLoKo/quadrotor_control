#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <rl_env/HectorQuad.hh>

// Random initialization of position
HectorQuad::HectorQuad(Random &rand, int target/* = 100*/):
  noisy(false),
  s(2),
  rng(rand),
  TARGET(target),
  num_actions(3),
  MAX_HEIGHT(2*TARGET),
  pos(0, 0, s[0]),
  vel(0, 0, s[1])
{
  ros::NodeHandle node;
  int qDepth = 1;
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  ros::Subscriber quadrotor_state = node.subscribe("/ground_truth/state", qDepth, &HectorQuad::gazeboStateCallback, this, noDelay);
  reset();
}

HectorQuad::~HectorQuad() { }

void HectorQuad::gazeboStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  pos(2) = msg->pose.pose.position.z;
  vel(2) = msg->twist.twist.linear.z;
}

void HectorQuad::refreshState() {
  s[0] = (pos(2)-TARGET >= 0)?1:0;
  s[1] = (pos(2)-TARGET <= 0)?1:0;
}

const std::vector<float> &HectorQuad::sensation() {
  refreshState();
  return s;
}

int HectorQuad::getNumActions() {
  return num_actions;
}

bool HectorQuad::terminal() {
  return (TARGET-pos(2)==0);
}

// Called by env.cpp for next action
float HectorQuad::apply(int action) {
  geometry_msgs::Twist action_vel;

  switch(action) {
    case UP:
      action_vel.linear.z = +2;
      break;
    case DOWN:
      action_vel.linear.z = -2;
      break;
    case STAY:
      action_vel.linear.z = 0;
      break;
  }

  cmd_vel.publish(action_vel);

  return reward();
}

// Reward policy function
float HectorQuad::reward() {
  return -abs(TARGET - pos(2));
}

void HectorQuad::setSensation(std::vector<float> newS){
  if (s.size() != newS.size()){
    cerr << "Error in sensation sizes" << endl;
  }

  for (unsigned i = 0; i < newS.size(); i++){
    s[i] = newS[i];
  }
}

std::vector<experience> HectorQuad::getSeedings() {
  // return seedings
  std::vector<experience> seeds;
  reset();

  return seeds;
}

experience HectorQuad::getExp(float s0, float s1, int a){
  experience e;
  e.s.resize(2, 0.0);
  e.next.resize(2, 0.0);

  pos(2) = s0;
  vel(2) = s1;

  e.act = a;
  e.s = sensation();
  e.reward = apply(e.act);

  e.terminal = terminal();
  e.next = sensation();

  return e;
}

void HectorQuad::getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat) {
  // No clue what model to impleent here
}


void HectorQuad::getMinMaxReward(float* minR, float* maxR) {
  *minR = -(num_actions-1)/2;
  *maxR = (num_actions-1)/2;
}

void HectorQuad::reset() {
  // Should be resetting the system, but we don't have anything to
}
