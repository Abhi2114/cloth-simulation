#ifndef _LEAPFROG
#define _LEAPFROG

#include "PositionUpdate.h"
#include "VelocityUpdate.h"

class LeapFrog : public Solver {

private:

  PositionUpdate *ps1;
  VelocityUpdate *vs;
  PositionUpdate *ps2;

public:

  LeapFrog() {}

  ~LeapFrog() {
    delete ps1;
    delete vs;
    delete ps2;
  }

  LeapFrog(PositionUpdate *p1, VelocityUpdate *v,
           PositionUpdate *p2): ps1(p1), vs(v), ps2(p2) {

  }

  void setForce(const Vector &f) {
    // set the force for the velocity update
    vs->setForce(f);
  }

  // useful when you want to apply the 3 solvers individually
  const PositionUpdate* getFirstPositionUpdate() const { return ps1; }
  const VelocityUpdate* getVelocityUpdate() const { return vs; }
  const PositionUpdate* getSecondPositionUpdate() const { return ps2; }

  // state.first = position vector
  // state.second = velocity vector
  pair operator () (const pair &&state, double dt) const {

    // combine 2 positional solvers with a velocity solver to get the leapfrog
    pair state1 = (*ps1)(std::move(state), dt);
    pair state2 = (*vs)(std::move(state1), dt);
    pair state3 = (*ps1)(std::move(state2), dt);

    return state3;
  }
};

#endif
