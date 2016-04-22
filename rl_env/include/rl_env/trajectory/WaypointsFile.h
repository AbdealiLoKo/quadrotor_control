#ifndef _WAYPOINTS_FILE_H_
#define _WAYPOINTS_FILE_H_

#include <rl_common/core.hh>

#include <rl_env/trajectory/Waypoints.h>

class WaypointsFile: public Waypoints {
private:
  std::string filename;
public:
  WaypointsFile(std::string filename, bool _use_checkpoints = false);

  void create_waypoints();
};

#endif
