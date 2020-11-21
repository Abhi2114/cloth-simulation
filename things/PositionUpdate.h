#ifndef _POSITION_UPDATE
#define _POSITION_UPDATE

#include "Solver.h"

class PositionUpdate : public Solver {

public:

  PositionUpdate() {}

  virtual ~PositionUpdate() {}

  PositionUpdate(double timestep): Solver(timestep) {}

  // state.first = position vector
  // state.second = velocity vector
  // leave the velocity unchanged
  virtual pair operator () (const pair &&state, double dt) const {
    // printf("PU\n");
    return pair(state.first + state.second * timestep * dt, state.second);
  }
};

#endif
