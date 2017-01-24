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
