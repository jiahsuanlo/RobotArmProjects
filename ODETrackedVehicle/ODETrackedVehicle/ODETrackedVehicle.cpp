// ODETrackedVehicle.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <ode\ode.h>
#include <drawstuff\drawstuff.h>

#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#endif

//========================================
// define parameters
double tStep = 0.01;
static int ct = 0;

//=======================================
// define ODE systems
static dWorldID world;  

// define space
static dSpaceID space;  // main space
static dSpaceID groundSpace;  // ground sub-space
static dSpaceID objectSpace;  // object sub-space
static dSpaceID trackSpace;  // track sub-space

// contact and geometry
static dJointGroupID contactGroup;
static dGeomID ground;



int main()
{
    return 0;
}

