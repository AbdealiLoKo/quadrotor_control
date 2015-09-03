#include <ros/ros.h>
#include <gazebo/gazebo.hh>

namespace gazebo {
  class EnvHectorQuadWorld : public WorldPlugin {
  public:
    EnvHectorQuadWorld() : WorldPlugin() {
      printf("Hello World!\n");
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
      ros::NodeHandle node;
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(EnvHectorQuadWorld)
}
