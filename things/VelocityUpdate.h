#ifndef _VELOCITY_UPDATE
#define _VELOCITY_UPDATE

#include "Solver.h"

class VelocityUpdate : public Solver {

private:

  Vector force;

public:

  VelocityUpdate() {}

  virtual ~VelocityUpdate() {}

  VelocityUpdate(double timestep, const Vector &force):
                 Solver(timestep), force(force) {}

  VelocityUpdate(double timestep): Solver(timestep), force(Vector()) {}

  void setForce(const Vector &f) {
    force = f;
  }

  // state.first = position vector
  // state.second = velocity vector
  // leave the position unchanged
  pair operator () (const pair &&state, double dt) const {
    return pair(state.first, state.second + force * dt * timestep);
  }
};

#endif
