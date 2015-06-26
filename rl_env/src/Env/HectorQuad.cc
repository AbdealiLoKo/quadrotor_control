#include <unistd.h>

#include <geometry_msgs/Twist.h>
#include <rl_env/HectorQuad.hh>
#include <gazebo_msgs/GetModelState.h>


// Random initialization of position
HectorQuad::HectorQuad(Random &rand,
                       Eigen::Vector3d target/* = Eigen::Vector3d(0, 0, 0)*/,
                       int time_step_in_ms/* = 1000 */):
  s(2),
  rng(rand),
  target_pos(target),
  num_actions(3),
  pos(0, 0, s[0]),
  vel(0, 0, s[1]),
  last_pos(0, 0, s[0]),
  last_vel(0, 0, s[1]),
  time_step(time_step_in_ms)
{
  ros::NodeHandle node;
  int qDepth = 1;
  // Publishers
  cmd_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  // Services
  reset_world = node.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
  pause_physics = node.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  unpause_physics = node.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  get_model_state = node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  get_physics_properties = node.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");

  // Wait for gazebo's services
  std::cout << "Waiting for gazebo ..." << std::endl;
  ros::service::waitForService("/gazebo/reset_world", -1);
  ros::service::waitForService("/gazebo/pause_physics", -1);
  ros::service::waitForService("/gazebo/unpause_physics", -1);
  ros::service::waitForService("/gazebo/get_model_state", -1);
  ros::service::waitForService("/gazebo/get_physics_properties", -1);

  reset();
}

HectorQuad::~HectorQuad() { }

void HectorQuad::refreshState() {
  // Let gazebo run for some time
  unpause_physics.call(empty_msg);
  usleep(time_step);
  pause_physics.call(empty_msg);

  // Get the final state from gazebo
  gazebo_msgs::GetModelState getmodelstate_msg;
  getmodelstate_msg.request.model_name = "quadrotor";
  get_model_state.call(getmodelstate_msg);

  pos(0) = getmodelstate_msg.response.pose.position.x;
  pos(1) = getmodelstate_msg.response.pose.position.y;
  pos(2) = getmodelstate_msg.response.pose.position.z;

  // Save state
  s[0] = (pos(2)-target_pos(2) >= 0)?1:0;
  s[1] = (pos(2)-target_pos(2) <= 0)?1:0;
}

const std::vector<float> &HectorQuad::sensation() {
  refreshState();
  // std::cout << "State: " << s[0] << s[1] << std::endl;
  return s;
}

int HectorQuad::getNumActions() {
  return num_actions;
}

bool HectorQuad::terminal() {
  if (abs(target_pos(2) - pos(2)) < 0.2) {
    terminal_count += 1;
    if (terminal_count >= 1000) {
      return(true);
    }
  } else {
    terminal_count = 0;
  }
  return(false);
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
  // std::cout << "Action:" << action_vel.linear.z << " State:" << s[0] << ","
  //           << s[1] << " Pos:" << pos(2) << " Targ:" << target_pos(2)
  //           << " Reward:" << reward() << std::endl;

  cmd_vel.publish(action_vel);

  return reward();
}

// Reward policy function
float HectorQuad::reward() {
  if ( abs(target_pos(2) - pos(2)) < abs(target_pos(2) - last_pos(2)) ) {
    return 1;
  } else if ( abs(target_pos(2) - pos(2)) > abs(target_pos(2) - last_pos(2)) ) {
    return -1;
  }
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
  terminal_count = 0;
  // Reset the world
  reset_world.call(empty_msg);
  pause_physics.call(empty_msg);
  get_physics_properties.call(getphysicsproperties_msg);
}
