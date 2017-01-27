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

/* obtain 4x4 transformation matrix out of DM parameters*/
void tmDM(const DHParameter & dm, Eigen::Matrix4d & tm)
{
	double a = dm.a;
	double d = dm.d;
	double cth = std::cos(dm.theta);
	double sth = std::sin(dm.theta);
	double ca = std::cos(dm.alpha);
	double sa = std::sin(dm.alpha);

	tm << cth, -sth*ca, sth*sa, a*cth,
		sth, cth*ca, -cth*sa, a*sth,
		0, sa, ca, d,
		0, 0, 0, 1;
}

/* Obtain joint and link information from Denavit-Hartenburg parameters
output:
	jntAnchors: joint anchor locations
	jntAxes: joint axis
	linkLengths: link lengths
	linkCMs: positions of all links
*/
void obtainJointLinkInfoFromDH(const std::vector<DHParameter>& dmp, 
	std::vector<Point3>& jntAnchors, std::vector<Point3>& jntAxes, 
	std::vector<double>& linkLengths, std::vector<Point3>& linkCMs)
{
	// clear output vectors first
	jntAnchors.clear();
	jntAxes.clear();
	linkLengths.clear();
	linkCMs.clear();
	
	// loop through all links
	Eigen::Matrix4d tmat, tmat1;
	Point3 end_prev(0, 0, 0);
	Point3 axis(0, 0, 1);
	jntAxes.push_back(axis);
	jntAnchors.push_back(end_prev); // base joint anchor position
	Point3 cm;
	tmat = Eigen::Matrix4d::Identity();
	for (auto dm : dmp)
	{
		// transform now
		tmDM(dm, tmat1);
		tmat = tmat*tmat1;
		// cm position
		Point3 end(tmat(0, 3), tmat(1, 3), tmat(2, 3));
		cm = 0.5*(end + end_prev);
		end_prev = end;  // update
		// joint axis (z axis)
		axis.x = tmat(0, 2); axis.y = tmat(1, 2); axis.z = tmat(2, 2);
		// link length
		double lnkL = std::sqrt(dm.a*dm.a + dm.d*dm.d);

		// add to output
		jntAxes.push_back(axis);
		jntAnchors.push_back(end);
		linkCMs.push_back(cm);
		linkLengths.push_back(lnkL);
	}
}

void quaternionFromRM(const Eigen::Matrix3d & rm, Quaternion & quat)
{
	Quaternion out;
	out.e0 = 0.5*std::sqrt(rm.trace()+1.0);

	return out;
}
