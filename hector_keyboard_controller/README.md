# QUadrotor controller
Keyboard based controller for Twist messages

Publishes Twist messages to cmd_vel

Linear velocities control:
* x: a/d
* y: w/s
* z: UP/DOWN

## Pre requisites
 1. [ROS](http://wiki.ros.org/ROS/Installation) - We use ROS Indigo
 1. [hector_quadrotor](http://wiki.ros.org/hector_quadrotor) - or any other simulator

## Usage
 1. Create a catking workspace:
   1. `mkdir -p ~/catkin_ws/src`
   1. `cd ~/catkin_ws`
   1. `catkin_build`
 1. Download the hector_quadrotor package:
   1. `cd ~/catkin_ws/src`
   1. `git clone https://github.com/Quadrotor-RL/Twist-controller-keyboard.git`
   1. `cd ~/catkin_ws`
   1. `catkin_make`
 1. Run a simulator (like hector_quadrotor)
   1. If you haven't installed it yet, you can do it inside your catkin workspace using:
     1. `cd ~/catkin_ws`
     1. `wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/hydro-devel/tutorials.rosinstall`
     1. `catkin_make` - install any dependencies that may be needed here
   1. `rosrun hector_quadrotor_demo outdoor_flight_gazebo.launch`
   1. Wait for gazebo to load and the scene to be setup
 1. Run our twist-controller
   1. `source ~/catkin_ws/devel/setup.sh`
   1. `roslaunch quadrotor_controller keyboard.py`

