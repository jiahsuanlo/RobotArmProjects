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
enum class JointType {Prismatic,Revolute};

struct TransMat
{
	TransMat() 
	{
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				elem[i][j] = 0.0;
			}
		}
	}
	
	void getEigenMat(Eigen::Matrix4d &tm);

	double elem[4][4];
};
struct DHParameter
{
	DHParameter(double a, double alpha, double d, double theta, JointType jtype= JointType::Revolute) :
		a0(a), alpha0(alpha), d0(d), theta0(theta),
		a(a), alpha(alpha), d(d), theta(theta), jtype(jtype) {}	
	double a0, alpha0, d0, theta0;
	JointType jtype;
	TransMat tm;

	// setters
	void setD(double d) { this->d = d + d0; }
	void setTheta(double th) { theta = theta0 + th; }
	void updateTM();

	// getters
	double getA() const { return a; }
	double getAlpha() const{ return alpha; }
	double getD() const{ return d; }
	double getTheta() const{ return theta; };
	void getTM(Eigen::Matrix4d &tm) const;
private:
	double a;
	double alpha;
	double d;
	double theta;
};
struct Quaternion
{
	Quaternion(double e0, double e1, double e2, double e3) :
		e0(e0), e1(e1), e2(e2), e3(e3) {}
	Quaternion() :e0(0), e1(0), e2(0), e3(0) {}
	double e0;
	double e1;
	double e2;
	double e3;
};

void rotx(double theta, Eigen::Matrix3d &rmat);
void roty(double theta, Eigen::Matrix3d &rmat);
void rotz(double theta, Eigen::Matrix3d &rmat);
void rot_zyz(double thz, double thy, double thz1, Eigen::Matrix3d &rmat);
void rot_zyx(double thz, double thy, double thx, Eigen::Matrix3d &rmat);
void tmDH(const DHParameter &dh, Eigen::Matrix4d &tm);
void tmDHs(const std::vector<DHParameter> &dhs, Eigen::Matrix4d &tm);
void tmDHs(const std::vector<DHParameter> &dhs,int iFirst, int iLast, Eigen::Matrix4d &tm);
void JgDHs(const std::vector<DHParameter> &dhs, 
	Eigen::Matrix4d &tm_0n, Eigen::MatrixXd &Jg);
void JgRPInverse(const Eigen::MatrixXd &Jg, Eigen::MatrixXd &J_plus);
void JgDLSInverse(const Eigen::MatrixXd &Jg, double k, Eigen::MatrixXd &J_star);

void obtainJointLinkInfoFromDH(const std::vector<DHParameter> &dmp,
	std::vector<Point3> &jntAnchors, std::vector<Point3> &jntAxes,
	std::vector<double> &linkLengths, std::vector<Point3> &linkCMs);

// ===== quaternion ====
void quaternionFromRM(const Eigen::Matrix3d &rm, Quaternion &quat);
Point3 operator -(const Quaternion &q1, const Quaternion &q2);
std::ostream &operator<<(std::ostream &output, const Quaternion &q);



