#ifndef _PUREPURSUIT_FILE_H_
#define _PUREPURSUIT_FILE_H_

#include <rl_common/core.hh>

#include <rl_env/trajectory/PurePursuit.h>

class PurePursuitFile: public PurePursuit {
private:
  std::string filename;
public:
  PurePursuitFile(std::string filename, double lookahead);

  void create_waypoints();
};

#endif
