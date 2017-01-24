#pragma once

#include <ode/ode.h> 
#include <drawstuff/drawstuff.h>
#include <vector>
#include <BasicMath.h>
#include <cmath>
using namespace BM;

class Link
{
public:
	Link(dWorldID world, double m, double len, double r, Point3 pos)
		:wid(world), mass(m), pos(pos), length(len), radius(r)
	{
		createMass();
	}
	void createMass();
	void createGeom(dSpaceID space);

	dWorldID wid;
	dBodyID bid;
	dGeomID gid;
	Point3 pos;
	double mass;

	double length;
	double radius;
	
};

class Box
{
public:
	Box(dWorldID world, double m, double len, double wid, double ht, Point3 pos)
		:wid(world), mass(m), pos(pos)
		, length(len), width(wid), height(ht)
	{
		createMass();
	}
	void createMass();
	void createGeom(dSpaceID space);
	
	dWorldID wid;
	dBodyID bid;
	dGeomID gid;
	Point3 pos;
	double mass;

	double length;
	double width;
	double height;
};

class Cylinder
{
public:
	Cylinder(dWorldID world, double m, double len, double radius, Point3 pos)
		:wid(world), mass(m), pos(pos)
		, length(len), radius(radius)
	{
		createMass();
	}
	void createMass();
	void createGeom(dSpaceID space);

	dWorldID wid;
	dBodyID bid;
	dGeomID gid;
	Point3 pos;
	double mass;

	double length;
	double radius;
};

struct Joint
{
	Joint(double tarAng, Point3 anchor, Point3 axis)
		:targetAngle(tarAng), anchor(anchor), axis(axis)
		,upperBound(M_PI), lowerBound(-M_PI)
	{}
	void setBounds(double lb, double ub);

	dJointID jid;
	void boundJointTargetAngle();
	double targetAngle;
	Point3 anchor;
	Point3 axis;
	double upperBound;
	double lowerBound;
};


