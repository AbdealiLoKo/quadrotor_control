#include <rl_env/trajectory/PurePursuitFile.h>

PurePursuitFile::PurePursuitFile(std::string file, double lookahead) :
PurePursuit(lookahead) {
  filename=file;
  viz_points_size = 0.01;
}

void PurePursuitFile::create_waypoints() {
  std::ifstream file(filename.c_str());
  std::string line;
  int k;

  while(std::getline(file, line)) {
    k += 1;

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
