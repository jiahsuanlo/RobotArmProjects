#pragma once

#include <ode/ode.h> 
#include <drawstuff/drawstuff.h>
#include <vector>
#include <BasicMath.h>
#include <cmath>
#include <Eigen/Dense>

using namespace BM;

// ===== robot data storage =====
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

// ===== robot calculation functions =====
struct DMParameter
{
	DMParameter(double a, double alpha, double d, double theta) :
		a(a), alpha(alpha), d(d), theta(theta) {}
	double a;
	double alpha;
	double d;
	double theta;
};

void rotx(double theta, Eigen::Matrix3d &rmat);
void roty(double theta, Eigen::Matrix3d &rmat);
void rotz(double theta, Eigen::Matrix3d &rmat);
void rot_zyz(double thz, double thy, double thz1, Eigen::Matrix3d &rmat);
void tmDM(const DMParameter &dm, Eigen::Matrix4d &tm);
void obtainJointLinkInfoFromDM(const std::vector<DMParameter> &dmp,
	std::vector<Point3> &jntAnchors, std::vector<Point3> &jntAxes,
	std::vector<double> &linkLengths, std::vector<Point3> &linkCMs);




