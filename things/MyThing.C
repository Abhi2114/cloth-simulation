//-------------------------------------------------------
//
//  MyThing.C
//
//  PbaThing for a collection of particles
//  each doing a random walk.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#include "MyThing.h"
#include <cstdlib>
#include <GL/gl.h>   // OpenGL itself.
#include <GL/glu.h>  // GLU support library.
#include <GL/glut.h> // GLUT support library.
#include <iostream>

using namespace std;

using namespace pba;


MyThing::MyThing(const std::string nam) :
 PbaThingyDingy (nam)
{
    Reset();
    cout << name << " constructed\n";

    // initialize solver for the animation
    PositionUpdate *ps1 = new PositionUpdate(0.5);
    VelocityUpdate *v = new VelocityUpdate(1.0);
    PositionUpdate *ps2 = new PositionUpdate(0.5);
    // now construct the leapfrog
    solver = new LeapFrog(ps1, v, ps2);
}

// init the coefficients
double MyThing::Ks = 150.0;
double MyThing::Kf = 0.0003;
double MyThing::Ka = 0.0;
Vector MyThing::gravity = Vector(0, -0.01, 0);
double MyThing::radius = 1.0;
double MyThing::speed = 0.6;

MyThing::~MyThing(){
  delete solver;
}

void MyThing::Init( const std::vector<std::string>& args ) {}

void MyThing::Display()
{
   for(int i = 0; i < SIZE; ++i) {
    for (int j = 0; j < SIZE; ++j) {
      const ParticleState& particle = lattice[i][j];
      const Vector& P = particle.position;
      const Color& ci = particle.color;
      glColor3f( ci.red(), ci.green(), ci.blue() );
      glVertex3f( P.X(), P.Y(), P.Z() );
      // draw lines between the points
      glBegin(GL_LINES);
      if (j + 1 < SIZE) {  // east
        glVertex3f(P.X(), P.Y(), P.Z());
        const Vector& right = lattice[i][j + 1].position;
        glVertex3f(right.X(), right.Y(), right.Z());
      }
      if (i + 1 < SIZE) { // south
        glVertex3f(P.X(), P.Y(), P.Z());
        const Vector& down = lattice[i + 1][j].position;
        glVertex3f(down.X(), down.Y(), down.Z());
      }
      if (i + 1 < SIZE && j - 1 > -1) {  // south west
        glVertex3f(P.X(), P.Y(), P.Z());
        const Vector& sw = lattice[i + 1][j - 1].position;
        glVertex3f(sw.X(), sw.Y(), sw.Z());
      }
      if (i + 1 < SIZE && j + 1 < SIZE) {  // south east
        glVertex3f(P.X(), P.Y(), P.Z());
        const Vector& se = lattice[i + 1][j + 1].position;
        glVertex3f(se.X(), se.Y(), se.Z());
      }
      glEnd();
    }
   }
}

void MyThing::Keyboard( unsigned char key, int x, int y )
{
       PbaThingyDingy::Keyboard(key,x,y);

       if (key == 'G') {
         gravity *= 1.05;
         cout << "Gravity = " << gravity.__str__() << endl;
       }
       if (key == 'g') {
         gravity /= 1.05;
         cout << "Gravity = " << gravity.__str__() << endl;
       }
       if (key == 'k') {
         Ks = max(10.0, Ks - 10.0);
         cout << "spring strength = " << Ks << "\n";
       }
       if (key == 'K') {
         Ks = min(200.0, Ks + 10.0);
         cout << "spring strength = " << Ks << "\n";
       }
       if (key == 'o') {
         Kf = max(0.0, Kf - 0.0001);
         cout << "friction strength = " << Kf << "\n";
       }
       if (key == 'O') {
         Kf = min(0.001, Kf + 0.0001);
         cout << "friction strength = " << Kf << "\n";
       }
       if (key == 's') {
         speed = max(0.1, speed - 0.1);
         cout << "speed = " << speed << "\n";
       }
       if (key == 'S') {
         speed = min(1.0, speed + 0.1);
         cout << "speed = " << speed << "\n";
       }
       if (key == 'd') {
         radius = max(0.0, radius - 0.2);
         cout << "radius = " << radius << "\n";
       }
       if (key == 'D') {
         radius = min(5.0, radius + 0.2);
         cout << "radius = " << radius << "\n";
       }
       if (key == 'a') {
         Ka = max(0.0, Ka - 0.5);
         cout << "Area strength = " << Ka << "\n";
       }
       if (key == 'A') {
         Ka = min(30.0, Ka + 0.5);
         cout << "Area strength = " << Ka << "\n";
       }
       if (key == 'q') {
         // quit
         exit(0);
       }
}

// compute strut force for (i, j) for given neighbor (p, q)
Vector MyThing::ParticleState::computeStrutForceAB(
int i, int j,
int p, int q,
const pair states[SIZE][SIZE],
const ParticleState home[SIZE][SIZE]
) {

  pair current = states[i][j];  // a

  pair neighbor = states[p][q]; // b
  // diff
  Vector Xab = neighbor.first - current.first;
  // rest length
  double Lab = (home[p][q].position - home[i][j].position).magnitude();
  // get force!
  double strutCoefficient = Ks * (Xab.magnitude() - Lab);
  // normalize to get direction for strut force
  Xab.normalize();

  Vector strutForce = strutCoefficient * Xab;
  // now compute the friction force

  // get the component of the relative velocity in the direction of the strut force
  double frictionCoefficient = Kf * ((neighbor.second - current.second) * Xab);
  // get force!
  Vector frictionForce = frictionCoefficient * Xab;

  return strutForce - frictionForce;  // friction is in the op direction to the strut
}


// compute total strut force for particle (i, j)
Vector MyThing::ParticleState::computeStrutForce(
int i, int j,
const pair states[SIZE][SIZE],
const ParticleState home[SIZE][SIZE]
) {

  Vector strut;

  // north
  if (i - 1 > -1)
    strut += computeStrutForceAB(i, j, i - 1, j, states, home);

  // north east
  if (i - 1 > -1 && j + 1 < SIZE)
    strut += computeStrutForceAB(i, j, i - 1, j + 1, states, home);

  // east
  if (j + 1 < SIZE)
    strut += computeStrutForceAB(i, j, i, j + 1, states, home);

  // south east
  if (i + 1 < SIZE && j + 1 < SIZE)
    strut += computeStrutForceAB(i, j, i + 1, j + 1, states, home);

  // south
  if (i + 1 < SIZE)
    strut += computeStrutForceAB(i, j, i + 1, j, states, home);

  // south west
  if (i + 1 < SIZE && j - 1 > -1)
    strut += computeStrutForceAB(i, j, i + 1, j - 1, states, home);

  // west
  if (j - 1 > -1)
    strut += computeStrutForceAB(i, j, i, j - 1, states, home);

  // north west
  if (i - 1 > -1 && j - 1 > -1)
    strut += computeStrutForceAB(i, j, i - 1, j - 1, states, home);

  return strut;
}

Vector MyThing::ParticleState::computeAreaForceTriangle(
int i, int j,
int p, int q,
int x, int y,
const pair states[SIZE][SIZE],
const ParticleState home[SIZE][SIZE]
) {

  // current particle (i, j) home and current position
  Vector ch = home[i][j].position;
  Vector cc = states[i][j].first;

  // compute area difference
  // home and current positions for a and b
  Vector ah = home[p][q].position;
  Vector bh = home[x][y].position;

  Vector ac = states[p][q].first;
  Vector bc = states[x][y].first;

  double homeArea = ((ah - ch) ^ (bh - ch)).magnitude();
  double currentArea = ((ac - cc) ^ (bc - cc)).magnitude();

  // compute force for the current particle
  // direction is in the direction of the bisector used to form the centroid
  Vector direction = ((0.5 * (ac + bc)) - cc).unitvector();

  return (Ka * (currentArea - homeArea)) * direction;
}

// form triangles with the non-diagonal neighbors of (i, j)
Vector MyThing::ParticleState::computeAreaForce(
int i, int j,
const pair states[SIZE][SIZE],
const ParticleState home[SIZE][SIZE]
) {

    Vector area;

    // north west triangle
    if (i - 1 > -1 && j - 1 > -1)
      area += computeAreaForceTriangle(i, j, i - 1, j, i, j - 1, states, home);

    // north east triangle
    if (i - 1 > -1 && j + 1 < SIZE)
      area += computeAreaForceTriangle(i, j, i - 1, j, i, j + 1, states, home);

    // south east triangle
    if (i + 1 < SIZE && j + 1 < SIZE)
      area += computeAreaForceTriangle(i, j, i, j + 1, i + 1, j, states, home);

    // south west triangle
    if (i + 1 < SIZE && j - 1 > -1)
      area += computeAreaForceTriangle(i, j, i + 1, j, i, j - 1, states, home);

    return area;
}

// compute force for particle (i, j) in the lattice
// the indices are needed to get access to the neighbors
Vector MyThing::ParticleState::computeForce(
int i, int j,
const pair states[SIZE][SIZE],
const ParticleState home[SIZE][SIZE]
) {

  // total force = strut + area + bend + gravity
  Vector strut = computeStrutForce(i, j, states, home);
  Vector area = computeAreaForce(i, j, states, home);

  return strut + area + gravity;
}

// simple routine to check if corner particle
bool isCornerParticle(int i, int j, const int SIZE) {

  return // (i == 0 && j == 0) || // top left
         (i == SIZE - 1 && j == 0) || // bottom left
         (i == 0 && j == SIZE - 1) || // top right
         (i == SIZE - 1 && j == SIZE - 1); // bottom right
}

void MyThing::solve()
{
   // This is where the particle state - position and velocity - are updated
   // in several steps.

   // init the count for the angle of rotation
   static double angle = 0.0;

   // keep track of all states
   pair states[SIZE][SIZE];

   const PositionUpdate *ps1 = solver->getFirstPositionUpdate();

   // partial position update # 1
   for (int i = 0; i < SIZE; ++i) {
     for (int j = 0; j < SIZE; ++j) {
       // move the first particle in circular motion
       if (i == 0 && j == 0) {

         // rotate in the x-z plane
         double y = home[i][j].position.Y() + radius * sin(speed * angle * 0.5);
         double x = (home[i][j].position.X() - radius) + radius * cos(speed * angle * 0.5);

         Vector current(x, y, home[i][j].position.Z());

         states[i][j] = pair(current, Vector(0, 0, 0));

         // incr angle
         angle += dt * 0.5;
       }
       else {
         pair pstate(lattice[i][j].position, lattice[i][j].velocity);
         states[i][j] = (*ps1)(std::move(pstate), dt);
       }
    }
   }

   const VelocityUpdate *vs = solver->getVelocityUpdate();

   // maintain a list of new forces to be applied to each particle
   Vector forces[SIZE][SIZE] = {Vector()};

   for (int i = 0; i < SIZE; ++i) {
     for (int j = 0; j < SIZE; ++j) {
        // do not apply for the corner particles since they are always fixed
        if ( !isCornerParticle(i, j, SIZE))
          // get force for particle (i, j)
          forces[i][j] = ParticleState::computeForce(i, j, states, home);
     }
   }

   for (int i = 0; i < SIZE; ++i) {
     for (int j = 0; j < SIZE; ++j) {
       // set force for the current particle
       solver->setForce(forces[i][j]);
       // velocity update
       states[i][j] = (*vs)(std::move(states[i][j]), dt);
     }
   }

   const PositionUpdate *ps2 = solver->getSecondPositionUpdate();

   for (int i = 0; i < SIZE; ++i) {
     for (int j = 0; j < SIZE; ++j) {
       // partial position update # 2
       // move the first particle in circular motion
       if (i == 0 && j == 0) {
         // get current position
         Vector& current = states[i][j].first;
         // rotate in the x-z plane
         double y = current.Y() + radius * sin(speed * angle * 0.5);
         double x = current.X() + radius * cos(speed * angle * 0.5);

         current.set(x, y, current.Z());

         // incr angle
         angle += dt * 0.5;
       }
       else
         states[i][j] = (*ps2)(std::move(states[i][j]), dt);

       // update the actual particle states now
       lattice[i][j].position = states[i][j].first;
       lattice[i][j].velocity = states[i][j].second;
     }
   }

}

void MyThing::Reset()
{
   cout << "Setting up\n";

   // start
   Vector startPosition(-43.0, -12.0, -43.0);
   Vector endPosition(42.0, -12.0, 42.0);
   // increment factor
   int incrementX = (endPosition.X() - startPosition.X()) / SIZE;
   int incrementZ = (endPosition.Z() - startPosition.Z()) / SIZE;

   for (int i = 0; i < SIZE; ++i) {
     for (int j = 0; j < SIZE; ++j) {
       // compute position
       lattice[i][j].position = Vector(startPosition.X() + i * incrementX,
                                       startPosition.Y(),
                                       startPosition.Z() + j * incrementZ
                                      );
       lattice[i][j].color = pba::Color(drand48(),drand48(),drand48(),0);
       lattice[i][j].velocity = Vector(0, 0, 0);
       // copy into home
       home[i][j] = lattice[i][j];
     }
   }

}

void MyThing::Usage()
{
   PbaThingyDingy::Usage();
   cout << "=== " << name << " ===\n";
   cout << "e            toggle particle emission on/off\n";
}


pba::PbaThing pba::CreateMyThing(){ return PbaThing( new MyThing() ); }
