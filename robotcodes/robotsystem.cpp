#include "robotsystem.h"

void Joint::setBounds(double lb, double ub)
{
	lowerBound = lb;
	upperBound = ub;
}

void Joint::boundJointTargetAngle()
{
	targetAngle = std::min(std::max(targetAngle, lowerBound), upperBound);
}

void Link::createMass()
{
	dMass tmpM;
	bid = dBodyCreate(wid);

	dBodySetPosition(bid,pos.x,pos.y,pos.z); // set position;
	dMassSetZero(&tmpM);      // initialize the mass parameter;
	dMassSetCapsuleTotal(&tmpM, mass, 3, radius, length);  // calculate the mass parameter;
	dBodySetMass(bid, &tmpM);  // set the mass parameter to the body;
	
}

void Link::createGeom(dSpaceID space)
{
	gid = dCreateCapsule(space, radius, length);
	dGeomSetPosition(gid, pos.x, pos.y, pos.z);
	dGeomSetBody(gid, bid);
}

void Box::createMass()
{
	dMass tmpM;
	bid = dBodyCreate(wid);

	dBodySetPosition(bid, pos.x, pos.y, pos.z); // set position;
	dMassSetZero(&tmpM);      // initialize the mass parameter;
	dMassSetBoxTotal(&tmpM, mass, length, width, height);  // calculate the mass parameter;
	dBodySetMass(bid, &tmpM);  // set the mass parameter to the body;
}

void Box::createGeom(dSpaceID space)
{
	gid = dCreateBox(space, length, width, height);
	dGeomSetPosition(gid, pos.x, pos.y, pos.z);
	dGeomSetBody(gid, bid);
}

void Cylinder::createMass()
{
	dMass tmpM;
	bid = dBodyCreate(wid);

	dBodySetPosition(bid, pos.x, pos.y, pos.z); // set position;
	dMassSetZero(&tmpM);      // initialize the mass parameter;
	dMassSetCylinderTotal(&tmpM, mass, 3, radius,length);  // calculate the mass parameter;
	dBodySetMass(bid, &tmpM);  // set the mass parameter to the body;
}

void Cylinder::createGeom(dSpaceID space)
{
	gid = dCreateCylinder(space, radius, length);
	dGeomSetPosition(gid, pos.x, pos.y, pos.z);
	dGeomSetBody(gid, bid);
}
// rotation matrix about x axis
void rotx(double theta, Eigen::Matrix3d & rmat)
{
	double cth = std::cos(theta);
	double sth = std::sin(theta);
	rmat << cth, -sth, 0,
		sth, cth, 0,
		0, 0, 1;
}
// rotation matrix about y axis
void roty(double theta, Eigen::Matrix3d & rmat)
{
	double cth = std::cos(theta);
	double sth = std::sin(theta);
	rmat << cth, 0, sth,
		0, 1, 0,
		-sth, 0, cth;
}
// rotation matrix about z axis
void rotz(double theta, Eigen::Matrix3d & rmat)
{
	double cth = std::cos(theta);
	double sth = std::sin(theta);
	rmat << cth, -sth, 0,
		sth, cth, 0,
		0, 0, 1;
}

void rot_zyz(double thz, double thy, double thz1, Eigen::Matrix3d & rmat)
{
	rmat << -sin(thz)*sin(thz1) + cos(thy)*cos(thz)*cos(thz1), -(sin(thz)*cos(thz1) + sin(thz1)*cos(thy)*cos(thz)), sin(thy)*cos(thz),
		sin(thz)*cos(thy)*cos(thz1) + sin(thz1)*cos(thz), -sin(thz)*sin(thz1)*cos(thy) + cos(thz)*cos(thz1), sin(thy)*sin(thz),
		-sin(thy - thz1), cos(thy - thz1), 0;
}
