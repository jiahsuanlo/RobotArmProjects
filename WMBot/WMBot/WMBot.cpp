// WMBot.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <robotsystem.h>
#include<iostream>

#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#endif

double tStep = 0.01;
static dWorldID world;         // a dynamic world;

static dSpaceID space;
static dSpaceID groundSpace;
static dSpaceID objectSpace;
static dSpaceID gripperSpace;
static dJointGroupID contactGroup;

static dGeomID ground;

static std::vector<Joint> joints;     // links, link[0] is a base;
static std::vector<Joint> gjoints;  // gripper joints
static std::vector<Link> links;    // joints, joint[0] is a fixed joint;  joint[0]
static std::vector<Box> gripperParts;

static std::vector<Box> boxes;
static std::vector<Link> caps;
static std::vector<Cylinder> cylinders;

dJointFeedback *feedback = new dJointFeedback;

// === rigidbody modeling functions
void configureLinks();  // robot arms
void configureGripper();  // gripper 
void configureObjects(); // object to be picked up

// === joint modeling functions
void configureArmJoints(); // arm joints
void configureGripperJoints(); // gripper joints

// === control functions 
void jointControl(); // arm joint controls
void gripperControl(); // gripper joint controls
void gripperControl1();

// === callbacks
static void nearCallback(void *data, dGeomID g1, dGeomID g2); // contact 
void start();  // ds draw start - camera view settings
void command(int cmd);  // ds callback command - user input
static void simLoop(int pause); // ds callback simLoop - simulation loop

								// === drawing functions
void drawArms(); // draw arms and gripper
void drawObjects();  // draw objects


// =============================================
// main function 
int mainOld(int argc, char *argv[])
{
	dsFunctions fn; // an variable for drawstuff;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.stop = NULL;
	fn.command = &command;
	fn.path_to_textures = "c:/dev/ode-0.13/drawstuff/textures";

	dInitODE();  // Initalize ODE;
	world = dWorldCreate();  // create a dynamic world;
	dWorldSetGravity(world, 0, 0, -9.81); // set gravity;

	//=============================
	// set spaces
	space = dHashSpaceCreate(0);
	groundSpace = dHashSpaceCreate(space);
	gripperSpace = dHashSpaceCreate(space);
	objectSpace = dHashSpaceCreate(space);
	dSpaceSetSublevel(groundSpace, 1);
	dSpaceSetSublevel(gripperSpace, 1);
	dSpaceSetSublevel(objectSpace, 1);

	contactGroup = dJointGroupCreate(0);
	// create ground
	ground = dCreatePlane(groundSpace, 0.0, 0.0, 1.0, 0.0);  // z= 0 plane
	
	configureLinks();
	configureGripper();
	configureObjects();
	//configureBoxes();

	configureArmJoints();
	configureGripperJoints();

	dsSimulationLoop(argc, argv, 1024, 768, &fn); // simulation loop;

	dWorldDestroy(world);
	dCloseODE(); // close ODE

	delete feedback;
	return 0;
}

int main(int argc, char *argv[])
{
	dsFunctions fn; // an variable for drawstuff;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.stop = NULL;
	fn.command = &command;
	fn.path_to_textures = "c:/dev/ode-0.13/drawstuff/textures";

	dInitODE();  // Initalize ODE;
	world = dWorldCreate();  // create a dynamic world;
	dWorldSetGravity(world, 0, 0, -9.81); // set gravity;

	//=============================
	// set spaces
	space = dHashSpaceCreate(0);
	groundSpace = dHashSpaceCreate(space);
	gripperSpace = dHashSpaceCreate(space);
	objectSpace = dHashSpaceCreate(space);
	dSpaceSetSublevel(groundSpace, 1);
	dSpaceSetSublevel(gripperSpace, 1);
	dSpaceSetSublevel(objectSpace, 1);

	contactGroup = dJointGroupCreate(0);
	// create ground
	ground = dCreatePlane(groundSpace, 0.0, 0.0, 1.0, 0.0);  // z= 0 plane

	configureLinks();
	configureGripper();
	configureObjects();
	//configureBoxes();

	configureArmJoints();
	configureGripperJoints();

	dsSimulationLoop(argc, argv, 1024, 768, &fn); // simulation loop;

	dWorldDestroy(world);
	dCloseODE(); // close ODE

	delete feedback;
	return 0;
}

// =============================================

int mainTest(int argc, char *argv[])
{
	std::vector<DMParameter> dms;
	dms.push_back(DMParameter(0, -0.5*M_PI, 10, 0));
	dms.push_back(DMParameter(10, 0, 0 ,-0.5*M_PI));

	std::vector<Point3> jntAnc, jntAx;
	std::vector<Point3> linkCM;
	std::vector<double> linkLen;
	obtainJointLinkInfoFromDM(dms, jntAnc, jntAx, linkLen, linkCM);

	// print
	for (int i = 0; i<int(jntAnc.size()); ++i)
	{
		std::cout << "joint " << i << " anchor: " << jntAnc[i];
		std::cout << "joint " << i << " axis: " << jntAx[i];
	}
	for (int i = 0; i<int(linkLen.size()); ++i)
	{
		std::cout << "link " << i+1 << " CM: " << linkCM[i];
		std::cout << "link " << i+1 << " length: " << linkLen[i]<<"\n";
	}


	std::system("pause");
	return 0;
}


void configureLinks()
{
	int nlink = 7;
	std::vector<double> masses = {20,10,5,5,3,2,2};
	std::vector<double> lengths = {0.1,0.2,0.4,0.3,0.1,0.1,0.1};
	std::vector<double> radius = {0.15,0.1, 0.05, 0.05,0.04,0.05,0.04};
		
	// initialize links (mass,len,radius,pos)
	for (int i = 0; i < nlink; ++i)
	{
		double z = 0.5*lengths[i];
		for (int j = 0; j < i; ++j)
		{
			z += lengths[j];
		}
		links.push_back(Link(world, masses[i], lengths[i], radius[i], Point3(0, 0, z)));
	}
}

void configureGripper()
{
	// base
	gripperParts.push_back(Box(world, 0.2, 0.05, 0.15, 0.01, Point3(0, 0, 1.35)));
	// left
	gripperParts.push_back(Box(world, 0.2, 0.05, 0.01, 0.15, Point3(0, 0.075, 1.45)));
	// right
	gripperParts.push_back(Box(world, 0.2, 0.05, 0.01, 0.15, Point3(0, -0.075, 1.45)));

	gripperParts[0].createGeom(gripperSpace);
	gripperParts[1].createGeom(gripperSpace);
	gripperParts[2].createGeom(gripperSpace);

}

void configureObjects()
{
	boxes.push_back(Box(world, 0.1, 0.05, 0.05, 0.2, Point3(0.0, 0.0, 1.45)));
	boxes[0].createGeom(objectSpace);

	caps.push_back(Link(world, 0.1, 0.2, 0.04, Point3(0.7, 0.0, 0.1)));
	caps[0].createGeom(objectSpace);

	cylinders.push_back(Cylinder(world, 0.1, 0.2, 0.04, Point3(0.0, 0.7, 0.1)));
	cylinders[0].createGeom(objectSpace);
}

void configureArmJoints()
{
	// initialize (tarAng, anchor, axis)	
	joints.push_back(Joint(0.0, Point3(0, 0, 0), Point3(0, 0, 1)));   // fixed joint
	joints.push_back(Joint(0.0, Point3(0, 0, links[0].pos.z + links[0].length*0.5), Point3(0, 0, 1)));  // rev joint
	joints.push_back(Joint(0.0, Point3(0, 0, links[1].pos.z + links[1].length*0.5), Point3(0, 1, 0)));
	joints.push_back(Joint(0.0, Point3(0, 0, links[2].pos.z + links[2].length*0.5), Point3(0, 1, 0)));
	joints.push_back(Joint(0.0, Point3(0, 0, links[3].pos.z + links[3].length*0.5), Point3(0, 0, 1)));
	joints.push_back(Joint(0.0, Point3(0, 0, links[4].pos.z + links[4].length*0.5), Point3(0, 1, 0)));
	joints.push_back(Joint(0.0, Point3(0, 0, links[5].pos.z + links[5].length*0.5), Point3(0, 0, 1)));

	// set joint bound
	joints[1].setBounds(-0.8*M_PI, 0.8*M_PI);
	joints[2].setBounds(-80*M_PI/180, 80*M_PI/180);
	joints[3].setBounds(-100 * M_PI / 180, 100 * M_PI / 180);
	joints[4].setBounds(-175 * M_PI / 180, 175 * M_PI / 180);
	joints[5].setBounds(-100 * M_PI / 180, 100 * M_PI / 180);
	joints[6].setBounds(-175 * M_PI / 180, 175 * M_PI / 180);

	// fixed joint
	joints[0].jid = dJointCreateFixed(world, 0); // a fixed joint;
	dJointAttach(joints[0].jid, links[0].bid, 0);     // attach the fixed joint to the ground;
	dJointSetFixed(joints[0].jid); // set the fixed joint;

	// from second joints
	int nj = int(joints.size());
	Point3 anc, ax;
	for (int j = 1; j <nj; ++j)
	{
		anc = joints[j].anchor;
		ax = joints[j].axis;
		joints[j].jid = dJointCreateHinge(world, 0);     // create a hinge joint;
		dJointAttach(joints[j].jid, links[j - 1].bid, links[j].bid); // attach the joints;
		dJointSetHingeAnchor(joints[j].jid, anc.x, anc.y, anc.z); // set an anchor point;
		dJointSetHingeAxis(joints[j].jid, ax.x, ax.y, ax.z); // set an rotation axis;
	}
}

void configureGripperJoints()
{
	gjoints.push_back(Joint(0, Point3(0, 0, 1), Point3(0, 0, 1))); // base
	gjoints.push_back(Joint(0, Point3(0, 0.075, 1), Point3(0, 1, 0))); // left
	gjoints.push_back(Joint(0, Point3(0, -0.075, 1), Point3(0, -1, 0))); // right

																		 // base
	gjoints[0].jid = dJointCreateFixed(world, 0);
	dJointAttach(gjoints[0].jid, links[6].bid, gripperParts[0].bid);
	dJointSetFixed(gjoints[0].jid);
	// left
	gjoints[1].jid = dJointCreateSlider(world, 0);
	dJointAttach(gjoints[1].jid, gripperParts[0].bid, gripperParts[1].bid);
	dJointSetSliderAxis(gjoints[1].jid
		, gjoints[1].axis.x, gjoints[1].axis.y, gjoints[1].axis.z);
	dJointSetSliderParam(gjoints[1].jid, dParamLoStop, -0.05);
	dJointSetSliderParam(gjoints[1].jid, dParamHiStop, 0.1);
	// right
	gjoints[2].jid = dJointCreateSlider(world, 0);
	dJointAttach(gjoints[2].jid, gripperParts[0].bid, gripperParts[2].bid);
	dJointSetSliderAxis(gjoints[2].jid
		, gjoints[2].axis.x, gjoints[2].axis.y, gjoints[2].axis.z);
	dJointSetSliderParam(gjoints[2].jid, dParamLoStop, -0.05);
	dJointSetSliderParam(gjoints[2].jid, dParamHiStop, 0.1);
}

void jointControl()
{
	double k1 = 10.0, fMax = 300.0; // k1:gain, fMax: max torque[Nm]

	int nj = int(joints.size());
	double angNow;
	double angErr;
	for (int i = 1; i != nj; ++i)
	{
		angNow = dJointGetHingeAngle(joints[i].jid);  // current joint angle; [rad]
		angErr = joints[i].targetAngle - angNow;  // target - current; 
		dJointSetHingeParam(joints[i].jid, dParamVel, k1*angErr); // angular velocity;
		dJointSetHingeParam(joints[i].jid, dParamFMax, fMax); // max torque;
	}
}

void gripperControl()
{
	double k1 = 10.0, fMax = 100.0; // k1:gain, fMax: max force[N]

	double sNow;
	double sErr;
	int nj = int(gjoints.size());
	for (int j = 1; j != nj; ++j)
	{
		sNow = dJointGetSliderPosition(gjoints[j].jid);  // current joint angle; [rad]
		sErr = gjoints[j].targetAngle - sNow;  // target - current; 

		dJointSetSliderParam(gjoints[j].jid, dParamVel, k1*sErr); // angular velocity;
		dJointSetSliderParam(gjoints[j].jid, dParamFMax, fMax); // max torque;
	}

}

void gripperControl1()
{
	double k1 = 10.0, fMax = 1000.0; // k1:gain, fMax: max force[N]

	double sNow;
	double sErr;
	double jfrc;
	int nj = int(gjoints.size());
	for (int j = 1; j != nj; ++j)
	{
		sNow = dJointGetSliderPosition(gjoints[j].jid);  // current joint angle; [rad]
		sErr = gjoints[j].targetAngle - sNow;  // target - current; 

		jfrc = 100.0*(2 * int(gjoints[j].targetAngle > 0) - 1);
		dJointAddSliderForce(gjoints[j].jid, jfrc);

		//dJointSetSliderParam(gjoints[j].jid, dParamVel, k1*sErr); // angular velocity;
		//dJointSetSliderParam(gjoints[j].jid, dParamFMax, fMax); // max torque;
	}

}


static void nearCallback(void *data, dGeomID g1, dGeomID g2)
{
	const int maxContacts = 4;
	dContact contact[maxContacts];

	if (dGeomIsSpace(g1) || dGeomIsSpace(g2))
	{
		// colliding a space with something
		dSpaceCollide2(g1, g2, data, &nearCallback);

		// collide all geoms internal to the space(s)
		if (dGeomIsSpace(g1))
			dSpaceCollide((dSpaceID)g1, data, &nearCallback);
		if (dGeomIsSpace(g2))
			dSpaceCollide((dSpaceID)g2, data, &nearCallback);
	}
	else
	{
		// check contacts
		int nContact = dCollide(g1, g2, maxContacts, &contact[0].geom, sizeof(dContact));

		// treat contacts
		if (nContact>0)
		{
			bool boxYes = ((g1 == boxes[0].gid) && (g2 == gripperParts[1].gid))
				|| ((g2 == boxes[0].gid) && (g1 == gripperParts[1].gid));
			bool capYes = ((g1 == caps[0].gid) && (g2 == gripperParts[1].gid))
				|| ((g2 == caps[0].gid) && (g1 == gripperParts[1].gid));
			bool cylYes = ((g1 == cylinders[0].gid) && (g2 == gripperParts[1].gid))
				|| ((g2 == cylinders[0].gid) && (g1 == gripperParts[1].gid));

			double fx = 0, fy = 0, fz = 0;
			//std::cout << "number of contacts: " << nContact <<std::endl;
			for (int i = 0; i<nContact; ++i)
			{
				contact[i].surface.mode = dContactApprox1_1 | dContactApprox1_2
					| dContactSoftCFM | dContactSoftERP;

				contact[i].surface.mu = 1;
				contact[i].surface.mu2 = 1;
				//contact[i].surface.slip1 = 1e-10;
				//contact[i].surface.slip2 = 1e-10;
				contact[i].surface.soft_erp = 0.01;   //0.01
				contact[i].surface.soft_cfm = 1e-5; //1e-5

													// build contact joint now
				dJointID c = dJointCreateContact(world, contactGroup, &contact[i]);
				dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
				//dJointAttach (c,dGeomGetBody(contact[i].geom.g1),dGeomGetBody(contact[i].geom.g2));
				if (boxYes)
				{
					dJointSetFeedback(c, feedback);
					feedback = dJointGetFeedback(c);
					fx += feedback->f1[0];
					fy += feedback->f1[1];
					fz += feedback->f1[2];
				}
			}
			if (boxYes)
				std::cout << "fx= " << fx << " fy= " << fy << " fz= " << fz << std::endl;
		}

	}
}

void start()
{
	float xyz[3] = { 3.04, 1.28, 0.76 };   // view point; [m]
	float hpr[3] = { -160.0, 4.50, 0.00 };  // view direction; (heading, pitch, roll) [deg]
	dsSetViewpoint(xyz, hpr);               // setting of view point and direction;
}

void command(int cmd)
{
	switch (cmd)
	{
	case 'q':
		joints[1].targetAngle += 0.05; //THETA[1] increment 0.05[rad]
									   // increases THETA[1] when j key is pressed
		break;
	case 'a':
		joints[1].targetAngle -= 0.05;
		break;
	case 'w':
		joints[2].targetAngle += 0.05;
		break;
	case 's':
		joints[2].targetAngle -= 0.05;
		break;
	case 'e':
		joints[3].targetAngle += 0.05;
		break;
	case 'd':
		joints[3].targetAngle -= 0.05;
		break;
	case 'r':
		joints[4].targetAngle += 0.05;
		break;
	case 'f':
		joints[4].targetAngle -= 0.05;
		break;
	case 't':
		joints[5].targetAngle += 0.05;
		break;
	case 'g':
		joints[5].targetAngle -= 0.05;
		break;
	case 'y':
		joints[6].targetAngle += 0.05;
		break;
	case 'h':
		joints[6].targetAngle -= 0.05;
		break;
	case 'u':
		if (gjoints[1].targetAngle > 0)
			gjoints[1].targetAngle = 0.0;
		else
			gjoints[1].targetAngle = 0.05;

		if (gjoints[2].targetAngle > 0)
			gjoints[2].targetAngle = 0.0;
		else
			gjoints[2].targetAngle = 0.05;


	}

	// English: limit target angles not to destroy a robot
	for (auto &jnt : joints)
	{
		jnt.boundJointTargetAngle();
	}
	//std::cout << joints[3].targetAngle << std::endl;
}

static void simLoop(int pause)
{
	dSpaceCollide(space, 0, &nearCallback);

	jointControl();
	gripperControl();

	dWorldStep(world, tStep);

	// collision setup
	dJointGroupEmpty(contactGroup);


	// draw arms and gripper
	drawArms();

	// draw objects
	drawObjects();
}

void drawArms()
{
	// define colors
	std::vector<double> rc = { 1,0,0,1,0,0.25,1 };
	std::vector<double> gc = { 0,1,0,1,1, 0.5,1 };
	std::vector<double> bc = { 0,0,1,0,1,   0,1 };

	// draw links
	int nl = int(links.size());
	int i = 0;
	for (auto &lk : links)
	{
		dsSetColor(rc[i], gc[i], bc[i]); // set color;
		dsDrawCapsuleD(dBodyGetPosition(lk.bid)
			, dBodyGetRotation(lk.bid), lk.length, lk.radius);
		i++;
	}

	// draw gripper
	dsSetColor(1.0, 0.0, 0.0); // set color;
	double sides[3];
	for (auto gp : gripperParts)
	{
		sides[0] = gp.length;
		sides[1] = gp.width;
		sides[2] = gp.height;
		dsDrawBoxD(dBodyGetPosition(gp.bid), dBodyGetRotation(gp.bid), sides);
	}
}

void drawObjects()
{
	// draw boxes
	dsSetColor(0.0, 1.0, 0.0); // set color;
	double sides[3];
	for (auto bx : boxes)
	{
		sides[0] = bx.length;
		sides[1] = bx.width;
		sides[2] = bx.height;

		dsDrawBoxD(dBodyGetPosition(bx.bid), dBodyGetRotation(bx.bid), sides);
	}

	// draw capsules
	dsSetColor(0.0, 1.0, 0.0); // set color;
	for (auto bx : caps)
	{
		dsDrawCapsuleD(dBodyGetPosition(bx.bid), dBodyGetRotation(bx.bid)
			, bx.length, bx.radius);
	}

	// draw cylinders
	dsSetColor(0.0, 1.0, 1.0); // set color;
	for (auto bx : cylinders)
	{
		dsDrawCylinderD(dBodyGetPosition(bx.bid), dBodyGetRotation(bx.bid)
			, bx.length, bx.radius);
	}
}
