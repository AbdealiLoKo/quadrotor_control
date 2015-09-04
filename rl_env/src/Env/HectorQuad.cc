#include <unistd.h>

#include <geometry_msgs/Twist.h>
#include <rl_env/HectorQuad.hh>


// Random initialization of position
HectorQuad::HectorQuad(Random &rand,
                       Eigen::Vector3d target/* = Eigen::Vector3d(0, 0, 5)*/):
  s(2),
  rng(rand),
  target_pos(target)
{
  ros::NodeHandle node;

  // Publishers
  cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

  // Services
  ros::service::waitForService("/gazebo/reset_world", -1);
  reset_world = node.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

  reset();
}

HectorQuad::~HectorQuad() {
}

const std::vector<float> &HectorQuad::sensation() {
  // Get state from gazebo

  // Convert gazebo's state to internal representation


  return s;
}

bool HectorQuad::terminal() {
  return(false);
}

float HectorQuad::apply(int action) {
  geometry_msgs::Twist action_vel;
  action_vel.linear.z = action;
  cmd_vel.publish(action_vel);
  return reward();
}

float HectorQuad::reward() {
  return -fabs(target_pos(2) - s[0]);
}

void HectorQuad::setSensation(std::vector<float> newS){
  if (s.size() != newS.size()){
    std::cerr << "Error in sensation sizes" << std::endl;
  }

  for (unsigned i = 0; i < newS.size(); i++){
    s[i] = newS[i];
  }
}

std::vector<experience> HectorQuad::getSeedings() {
  std::vector<experience> seeds;
  reset();
  return seeds;
}

experience HectorQuad::getExp(float s0, float s1, int a){
}

void HectorQuad::getMinMaxFeatures(std::vector<float> *minFeat, std::vector<float> *maxFeat) {
}

void HectorQuad::getMinMaxReward(float* minR, float* maxR) {
  *minR = -1;
  *maxR = -1;
}

void HectorQuad::reset() {
  reset_world.call(empty_msg); // Reset the world
}
