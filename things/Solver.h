#ifndef _SOLVER_H
#define _SOLVER_H

#include "Vector.h"

using namespace pba;

#include <utility> // for std::pair

#define pair std::pair<Vector, Vector>

// abstract Solver class
class Solver {

protected:
  // multiple of dt
  double timestep;

public:

  Solver(): timestep(0) {}

  virtual ~Solver() {}

  // for velocity updates!
  virtual void setForce(const Vector &f) {}

  Solver(double timestep): timestep(timestep) {}

  // a Solver doesn't know what type of Solver it is
  // so just use a pure virtual function
  // state = tuple of position and velocity, i.e state = (position, velocity)
  virtual pair operator () (const pair &&state, double dt) const = 0;

};

#endif
