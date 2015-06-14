#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <rl_env/HectorQuad.hh>

// Random initialization of position
HectorQuad::HectorQuad(Random &rand):
  noisy(false),
  s(2),
  rng(rand),
  TARGET(100),
  num_actions(9),
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
  // TODO: May need normalization
  s[0] = int(log(pos(2)));
  s[1] = int(log(vel(2)));
}

// zError function calculation
int HectorQuad::zError() {
  // Get the current Z position directly using a subscriber
  return (TARGET-pos(2));
}

const std::vector<float> &HectorQuad::sensation() {
  // Returns the current state
  // Characterised by Quad position in Z direction and velocity
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
  // Let's publish here to make the quad go up or down
  // TODO Use talker.py for actions

  // Finding z_desired velocity
  int z_desired = action - (num_actions-1)/2;

  // Publish a simple message with the z_desired
  geometry_msgs::Twist m;
  m.linear.z = z_desired;
  cmd_vel.publish(m);

  refreshState();
  return reward();
}

// Reward policy function
float HectorQuad::reward() {
  if (!zError())
    return 0;
  else
    // Return a negative reward in every other case
    return -log(abs(zError())+1);
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
