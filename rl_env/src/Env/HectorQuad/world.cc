#include <unistd.h>

#include <rl_common/core.hh>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <rl_common/RLRunSim.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#define DEBUG 0

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
    }

    bool do_run_sim(rl_common::RLRunSim::Request &req,
                    rl_common::RLRunSim::Response &res) {
      int step_count = req.steps;
      world_ptr->StepWorld(step_count);

      physics::ModelPtr quad_ptr = world_ptr->GetModel("quadrotor");
      if ( quad_ptr == NULL ) {
        res.success = false;
        return true;
      }

      // Use payload only if it exists.
      physics::LinkPtr link_ptr;
      physics::LinkPtr base_ptr = quad_ptr->GetLink("base_link");
      physics::LinkPtr payload_ptr = quad_ptr->GetLink("payload");

      if (payload_ptr) {
        link_ptr = payload_ptr;
        if (DEBUG) {
          ROS_INFO("Using PAYLOAD link for state in world.cc");
        }
      } else {
        link_ptr = base_ptr;
        if (DEBUG) {
          ROS_INFO("Using BASE_LINK link for state in world.cc");
        }
      }

      if (DEBUG) {
        if (base_ptr) {
          std::cout << "Quadrotor Base link\n"
                    << "Position = " << base_ptr->GetWorldPose().pos << "\n"
                    << "Orientation = " << base_ptr->GetWorldPose().rot << "\n"
                    << "Velocity = " << base_ptr->GetWorldLinearVel() << "\n"
                    << "Ang velocity = " << base_ptr->GetWorldAngularVel() << "\n";
        }
        if (payload_ptr) {
          std::cout << "Payload link\n"
                    << "Position = " << payload_ptr->GetWorldPose().pos << "\n"
                    << "Orientation = " << payload_ptr->GetWorldPose().rot << "\n"
                    << "Velocity = " << payload_ptr->GetWorldLinearVel() << "\n"
                    << "Ang velocity = " << payload_ptr->GetWorldAngularVel() << "\n";
        }
      }
      res.success = true;

      // Set time
      common::Time sim_time = world_ptr->GetSimTime();
      res.sim_time.sec = sim_time.sec;
      res.sim_time.nsec = sim_time.nsec;

      // Set pose
      math::Pose pose = link_ptr->GetWorldPose();
      res.pose.position.x = pose.pos.x;
      res.pose.position.y = pose.pos.y;
      res.pose.position.z = pose.pos.z;
      res.pose.orientation.x = pose.rot.x;
      res.pose.orientation.y = pose.rot.y;
      res.pose.orientation.z = pose.rot.z;
      res.pose.orientation.w = pose.rot.w;

      // Set twist
      math::Vector3 lin_vel = link_ptr->GetWorldLinearVel();
      res.twist.linear.x = lin_vel.x;
      res.twist.linear.y = lin_vel.y;
      res.twist.linear.z = lin_vel.z;
      math::Vector3 ang_vel = link_ptr->GetWorldAngularVel();
      res.twist.angular.x = ang_vel.x;
      res.twist.angular.y = ang_vel.y;
      res.twist.angular.z = ang_vel.z;
      // std::cout << "Time : " << sim_time.sec << "." << sim_time.nsec
      //           << "\t\tPos : " << pose.pos.z << "\n";

      return true;
    }

    ros::ServiceServer run_sim;
    physics::WorldPtr world_ptr;
  };
  GZ_REGISTER_WORLD_PLUGIN(EnvHectorQuadWorld)
}
