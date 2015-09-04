#include <unistd.h>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <rl_msgs/RLRunSim.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

namespace gazebo {
  class EnvHectorQuadWorld : public WorldPlugin {
  public:
    EnvHectorQuadWorld() : WorldPlugin() {}

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
      world_ptr = _world;
      ros::NodeHandle node;
      run_sim = node.advertiseService("rl_env/run_sim",
                                      &EnvHectorQuadWorld::do_run_sim,
                                      this);
      // world_ptr->SetPaused(true);
    }

    bool do_run_sim(rl_msgs::RLRunSim::Request &req,
                    rl_msgs::RLRunSim::Response &res) {
      int step_count = req.steps;
      world_ptr->StepWorld(step_count);

      physics::ModelPtr model_ptr = world_ptr->GetModel("quadrotor");
      if ( model_ptr == NULL ) {
        res.success = false;
        return true;
      } else {
        res.success = true;

        // Set time
        common::Time sim_time = world_ptr->GetSimTime();
        res.sim_time.sec = sim_time.sec;
        res.sim_time.nsec = sim_time.nsec;

        // Set pose
        math::Pose pose = model_ptr->GetWorldPose();
        res.pose.position.x = pose.pos.x;
        res.pose.position.y = pose.pos.y;
        res.pose.position.z = pose.pos.z;
        res.pose.orientation.x = pose.rot.x;
        res.pose.orientation.y = pose.rot.y;
        res.pose.orientation.z = pose.rot.z;
        res.pose.orientation.w = pose.rot.w;

        // Set twist
        math::Vector3 lin_vel = model_ptr->GetWorldLinearVel();
        res.twist.linear.x = lin_vel.x;
        res.twist.linear.y = lin_vel.y;
        res.twist.linear.z = lin_vel.z;
        math::Vector3 ang_vel = model_ptr->GetWorldAngularVel();
        res.twist.angular.x = ang_vel.x;
        res.twist.angular.y = ang_vel.y;
        res.twist.angular.z = ang_vel.z;

        return true;
      }
    }

    ros::ServiceServer run_sim;
    physics::WorldPtr world_ptr;
  };
  GZ_REGISTER_WORLD_PLUGIN(EnvHectorQuadWorld)
}
