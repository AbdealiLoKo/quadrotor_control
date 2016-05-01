#ifndef _WAYPOINTS_POINTS_H_
#define _WAYPOINTS_POINTS_H_

#include <rl_common/core.hh>

#include <rl_env/trajectory/Waypoints.h>
#include <rl_env/points/PointsBase.h>

template <class Type> class WaypointsPoints: public Waypoints  {
private:
	Type p;
public:
	WaypointsPoints(bool _use_checkpoints=false);
	void create_waypoints();
};

template <class Type>
WaypointsPoints<Type>::WaypointsPoints(bool _use_checkpoints) :
Waypoints(_use_checkpoints) {
  // From waypoints
  epsilon_x = epsilon_y = 1;
  epsilon_z = 1;
  epsilon_plane = 0.1;
}

template <class Type>
void WaypointsPoints<Type>::create_waypoints() {
  points = p.get_points();
}



#endif
