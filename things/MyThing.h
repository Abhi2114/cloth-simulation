//-------------------------------------------------------
//
//  MyThing.h
//
//  PbaThing for a collection of particles
//  each doing a random walk.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#include "Vector.h"
#include "Color.h"
#include "PbaThing.h"

#include "PositionUpdate.h"
#include "VelocityUpdate.h"
#include "LeapFrog.h"

using namespace std;

namespace pba{





class MyThing: public PbaThingyDingy
{
  public:

    // Feel free to customize the name of this thing.
    MyThing(const std::string nam = "Thing!");
   ~MyThing();

    //! Initialization, including GLUT initialization.
    //! Called once at the beginning.  Could be used
    //! to set up things once.
    void Init( const std::vector<std::string>& args );

    ///////////////////////////////////////////////////////////////
    // CASCADING CALLBACK FUNCTIONS
    // The methods below are called as part of a bigger set
    // of similar calls.  Most of the other calls take place
    // in the viewer portion of this project.
    ///////////////////////////////////////////////////////////////

    //! Implements a display event
    //! This is where you code the opengl calls to display
    //! your system.
    void Display();

    //! Implements responses to keyboard events
    //! This is called when you hit a key
    void Keyboard( unsigned char key, int x, int y );

    //! Implements simulator updates during an idle period
    //! This is where the update process is coded
    //! for your dynamics problem.
    void solve();

    //! Implements reseting parameters and/or state
    //! This is called when you hit the 'r' key
    void Reset();

    //! Displays usage information on stdout
    //! If you set up actions with the Keyboard()
    //! callback, you should include a statement
    //! here as to what the keyboard option is.
    void Usage();

    // size of grid forming the lattice
    static const int SIZE = 50;

    // constants for the cloth
    static double Ks;  // strut strength
    static double Kf;  // friction strength
    static double Ka;  // area strength
    static Vector gravity;    // force of gravity!
    static double radius;  // radius
    static double speed;   // speed of rotation

  private:

    ////////////////////////////////////////////////
    //
    //      PARTICLE STATE
    //
    // The state of a particle is characterized by
    // (1) particle positions
    // (2) particle velocities
    // (3) particle masses
    // (4) particle colors - useful for display
    class ParticleState
    {

      public:
        ParticleState() :
         position(Vector(0,0,0)),
      	 velocity(Vector(0,0,0)),
      	 color(Color(1,1,1,1)),
      	 mass(1.0) {};

       ~ParticleState(){};

       Vector position;
       Vector velocity;
       Color color;
       float mass;

       // compute total force for particle (i, j)
       static Vector computeForce(int i, int j,
                                  const pair states[SIZE][SIZE],
                                  const ParticleState home[SIZE][SIZE]);

       // compute strut force between 2 particles A(i, j) and B(p, q)
       static Vector computeStrutForceAB(int i, int j,
                                         int p, int q,
                                         const pair states[SIZE][SIZE],
                                         const ParticleState home[SIZE][SIZE]);

       // compute area force for particle (i, j) with the triangle formed
       // using vertices (i, j), (p, q) and (x, y)
       static Vector computeAreaForceTriangle(int i, int j,
                                              int p, int q,
                                              int x, int y,
                                              const pair states[SIZE][SIZE],
                                              const ParticleState home[SIZE][SIZE]);

       // compute area force for particle (i, j)
       static Vector computeAreaForce(int i, int j,
                                      const pair states[SIZE][SIZE],
                                      const ParticleState home[SIZE][SIZE]);

       // compute total strut force for particle (i, j)
       static Vector computeStrutForce(int i, int j,
                                       const pair states[SIZE][SIZE],
                                       const ParticleState home[SIZE][SIZE]);
    };

    // lattice used to represent the cloth, current state
    ParticleState lattice[SIZE][SIZE];
    // home (initial) state for the lattice
    ParticleState home[SIZE][SIZE];

    // solver!!
    LeapFrog *solver;

    //
    //
    ////////////////////////////////////////////////

};


// This function constructs the MyThing and wraps it in a
// smart pointer called a PbaThing.
// You need not alter this.
pba::PbaThing CreateMyThing();








}
