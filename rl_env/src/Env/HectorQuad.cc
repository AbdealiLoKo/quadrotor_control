#include <rl_env/Hector_quadrotor.hh>
#include <math.h>
#include <geometry_msgs/Twist.h>

// Random initialization of position
HectorQuad::HectorQuad(Random &rand):
  noisy(false),
  s(2),
  rng(rand),
  zPos(s[0]),
  zVel(s[1]),
  TARGET(100),
  num_actions(9),
  MAX_HEIGHT(2*TARGET)
{
  ros::NodeHandle nh;
  cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  ros::Subscriber quadrotor_state = node.subscribe("/altimeter", 1000, zPosCallback, &(this));
  reset();
}

HectorQuad::~HectorQuad() { }

void HectorQuad::zPosCallback(const hector_uav_msgs::Altimeter::ConstPtr& msg) {
  zVel = msg->altitude;
}

void HectorQuad::refreshState() {
  // TODO: May need normalization
  s[0] = int(log(zPos));
  s[1] = int(log(zVel));
}

// zError function calculation
int HectorQuad::zError() {
  // Get the current Z position directly using a subscriber
  return (TARGET-zPos);
}

const std::vector<float> &HectorQuad::sensation() const {
  // Returns the current state
  // Characterised by Quad position in Z direction and velocity
  return s;
}

int HectorQuad::getNumActions() {
  return num_actions;
}

bool HectorQuad::terminal() const {
  return (TARGET-zPos==0);
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

  zPos = s0;
  zVel = s1;

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
