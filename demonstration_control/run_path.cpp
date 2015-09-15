#include <ros/ros.h>
#include <iostream>

#include <rl_env/HectorQuad.hh>
#include <rl_common/Random.h>

#include <Eigen/Geometry>

#include <unistd.h>


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "DemoController");
  Random rng(1);

  Eigen::Vector3d initial_pos = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d final_pos = Eigen::Vector3d(10, 0, 0);

  HectorQuad h = HectorQuad(rng);

  int k = 1;
  ros::Rate r(100);  
  while (ros::ok()) {
    for(float i=-2; i<=2; i++) {
      std::cout<<"I is "<<i<<std::endl;
      int count = 0;
      int steps = 0;
      h.reset();
      Eigen::Vector3d dist = final_pos - initial_pos;
      Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);

      while(1) {
        if(dist.squaredNorm() < 0.001) {
          count++;
          if (count > 100)
            break;
        }
        else
          count = 0;
      
        // Don't use the Quadrotor reward, simply find the reward here
        dist = final_pos - h.get_pos();
        vel = dist * (k+i/10.0);
        
        // Give an action for the quadrotor
        h.set_vel(vel);
        ros::spinOnce();                 
        r.sleep();
        steps++;
      }
      std::cout<<"Reached, Steps used "<<steps<<std::endl;
      if (i == 2) {
        ros::shutdown();
      }
    }
  }
  return 0;
}
