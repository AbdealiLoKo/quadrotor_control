#ifndef _PURE_PURSUIT_POINTS_H_
#define _PURE_PURSUIT_POINTS_H_

#include <rl_common/core.hh>

#include <rl_env/trajectory/PurePursuit.h>
#include <rl_env/points/PointsBase.h>

template <class Type> class PurePursuitPoints: public PurePursuit  {
private:
	Type p;
public:
	PurePursuitPoints(double lookahead);
	void create_waypoints();
};

// Class needs to be implemented here to avoid linking errors
template <class Type>
PurePursuitPoints<Type>::PurePursuitPoints(double l) :
PurePursuit(l) {
}

template <class Type>
void PurePursuitPoints<Type>::create_waypoints() {
  points = p.get_points();
}

#endif
