#include <rl_env/trajectory/WaypointsFile.h>

WaypointsFile::WaypointsFile(std::string file, bool _use_checkpoints /*= false*/) :
Waypoints(_use_checkpoints) {
  filename=file;
  epsilon_plane=0.9;
}

void WaypointsFile::create_waypoints() {
  std::ifstream file(filename.c_str());
  std::string line;
  int k;

  while(std::getline(file, line)) {
    k += 1;

    // Use only 1/20 of the states since in a dense trajectory
    // the quadrotor gets decelerated too quickly.
    if (k%20 != 0) {
      continue;
    }

    std::stringstream linestream(line);
    std::string data;
    float x, y, z, vx, vy, vz, yaw;
    linestream >> x >> y >> z >> vx >> vy >> vz >> yaw;
    geometry_msgs::Point wp;
    wp.x = x;
    wp.y = y;
    wp.z = z;

    points.push_back(wp);
  }
}
