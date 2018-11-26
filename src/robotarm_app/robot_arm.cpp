// WMBot.cpp : Defines the entry point for the console application.
//

#include <ode/ode.h> 
#include <drawstuff/drawstuff.h>


#include <robotsystem.h>
#include<iostream>

#ifdef dDOUBLE
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCapsule dsDrawCapsuleD
#endif

double tStep = 0.01;
static int ct = 0;
static int ctrlMode = 0;  // 0: joint mode; 1: global mode; 2:end-effector mode
static dWorldID world;         // a dynamic world;

static dSpaceID space;
static dSpaceID groundSpace;
static dSpaceID objectSpace;
static dSpaceID gripperSpace;
static dJointGroupID contactGroup;

static dGeomID ground;

static std::vector<DHParameter> dhVec; // DH parameters
static std::vector<Joint> joints;     // links, link[0] is a base;
static std::vector<Joint> gjoints;  // gripper joints
static std::vector<Link> links;    // joints, joint[0] is a fixed joint;  joint[0]
static std::vector<Box> gripperParts;

static std::vector<Box> boxes;
static std::vector<Link> caps;
static std::vector<Cylinder> cylinders;

static Point3 posTarget;
static Point3 angTarget;

dJointFeedback *feedback = new dJointFeedback;

// === rigidbody modeling functions
void configureDHs(); // configure DH parameters
void configureLinks();  // robot arms
void configureGripper();  // gripper 
void configureObjects(); // object to be picked up

// === joint modeling functions
void configureArmJoints(); // arm joints
void configureGripperJoints(); // gripper joints

// === control functions 
void getJointAngles(std::vector<double> &angles);
void jointControl(); // arm joint controls
void gripperControl(); // gripper joint controls
void gripperControl1();

// === callbacks
static void nearCallback(void *data, dGeomID g1, dGeomID g2); // contact 
void start();  // ds draw start - camera view settings
void command(int cmd);  // ds callback command - user input
static void simLoop(int pause); // ds callback simLoop - simulation loop

void inverseKinematicsAuto();
void inverseKinematicsGlobal();
void inverseKinematicsEE();  // end-effector mode

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
	fn.path_to_textures = "c:/dev/ode-0.15.2/drawstuff/textures";

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
	fn.path_to_textures = "c:/dev/ode-0.15.2/drawstuff/textures";

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

	// hard code DH parameters
	configureDHs();

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
	
	configureDHs();

	// base tm
	Eigen::Matrix4d tmB0;
	tmB0 = Eigen::Matrix4d::Identity();
	tmB0(0, 3) = 0; tmB0(1, 3) = 0; tmB0(2, 3) = 0.3;

	Eigen::Matrix4d tm,tm1,tm2,tm3;
	// total 
	tmDHs(dhVec, tm);
	tm = tmB0*tm;

	
	std::cout << "tip= " << tm<<"\n";

	// debug 
	tmDH(dhVec[0], tm1);
	std::cout << "tm01:\n" << tmB0*tm1<<"\n";
	dhVec[1].setTheta(-0.5*M_PI);
	std::cout << "thz1= " << dhVec[1].getTheta() << "\n";
	tmDH(dhVec[1], tm2);
	std::cout << "tm02:\n" << tmB0*tm1*tm2 << "\n";
	
	std::system("pause");
	return 0;
}

void configureDHs()
{
	std::vector<double> ls = { 0.1,0.2,0.4,0.3,0.1,0.1,0.25 };
	dhVec.clear();
	dhVec.push_back(DHParameter(0, -0.5*M_PI, ls[0]+ls[1], 0)); // link1
	dhVec.push_back(DHParameter(ls[2],0,0,-0.5*M_PI));   // link2
	dhVec.push_back(DHParameter(0, 0.5*M_PI, 0, 0.5*M_PI));   // link3
	dhVec.push_back(DHParameter(0, -0.5*M_PI, ls[3] + ls[4], 0));   // link4
	dhVec.push_back(DHParameter(0, 0.5*M_PI, 0, 0));   // link5
	dhVec.push_back(DHParameter(0,0, ls[5]+ls[6], 0));   // link6
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

void getJointAngles(std::vector<double> &angles)
{
	// clear vector
	angles.clear();
	
	int nj = int(joints.size());
	double angNow;
	for (int i = 1; i != nj; ++i)
	{
		angNow = -dJointGetHingeAngle(joints[i].jid);  // current joint angle; [rad]
		angles.push_back(angNow);
	}
}

void jointControl()
{
	double k1 = 10.0, fMax = 400.0; // k1:gain, fMax: max torque[Nm]
	double k1d = 0.1;

	int nj = int(joints.size());
	double angNow;
	double angErr, angD;

	std::vector<double> angErrVec;
	for (int i = 1; i != nj; ++i)
	{
		angNow = -dJointGetHingeAngle(joints[i].jid);  // current joint angle; [rad]
		angD = -dJointGetHingeAngleRate(joints[i].jid);
		angErr = joints[i].targetAngle - angNow;  // target - current; 
		dJointSetHingeParam(joints[i].jid, dParamVel, -k1*angErr+k1d*angD); // angular velocity;
		dJointSetHingeParam(joints[i].jid, dParamFMax, fMax); // max torque;
		angErrVec.push_back(angErr);
	}
	

	if (ct % 1000 == 0)
	{
		std::cout << "Joint target angle: \n";
		for (int i = 0; i < 6; ++i)
		{
			std::cout << joints[i + 1].targetAngle << ",";
		}
		//std::cout << "angle= " << dhVec[1].getTheta() << " ODE angle= " << dJointGetHingeAngle(joints[2].jid) << "\n";
		//std::cout << "tip location= " << tm_0n(0, 3) << ", " << tm_0n(1, 3) << ", " << tm_0n(2, 3) << "\n";
		const dReal *pos = dBodyGetPosition(gripperParts[0].bid);
		std::cout << "\nODE tip location= " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		std::cout << "\nJointCtrl Err: ";
		std::for_each(angErrVec.begin(), angErrVec.end(),
			[](double x) {std::cout << x << ", "; });
		std::cout << "\n";
		//std::cout << " ODE tip location= " << pos[2] << "\n";
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
	angTarget= (1.0-10*tStep)*angTarget;
	posTarget = (1.0-10*tStep)*posTarget;

	double angDelta = 0.1;

	switch (cmd)
	{
	case 'z':
		ctrlMode++;
		ctrlMode = (ctrlMode % 3);
		break;
	case 'q':
		if (ctrlMode == 0)
			joints[1].targetAngle += 0.05; //THETA[1] increment 0.05[rad]
		else
			posTarget.x += 0.03; 
		break;
	case 'a':
		if (ctrlMode == 0)
			joints[1].targetAngle -= 0.05;
		else
			posTarget.x -= 0.03;
		break;
	case 'w':
		if (ctrlMode == 0)
			joints[2].targetAngle += 0.05;
		else
			posTarget.y += 0.03;
		break;
	case 's':
		if (ctrlMode == 0)
			joints[2].targetAngle -= 0.05;
		else
			posTarget.y -= 0.03;
		break;
	case 'e':
		if (ctrlMode == 0)
			joints[3].targetAngle += 0.05;
		else
			posTarget.z += 0.03;
		break;
	case 'd':
		if (ctrlMode == 0)
			joints[3].targetAngle -= 0.05;
		else
			posTarget.z -= 0.03;
		break;
	case 'r':
		if (ctrlMode == 0) joints[4].targetAngle += 0.05;
		else angTarget.x += angDelta;
		break;
	case 'f':
		if (ctrlMode == 0) joints[4].targetAngle -= 0.05;
		else angTarget.x -= angDelta;
		break;
	case 't':
		if (ctrlMode == 0) joints[5].targetAngle += 0.05;
		else angTarget.y += angDelta;
		break;
	case 'g':
		if (ctrlMode == 0) joints[5].targetAngle -= 0.05;
		else angTarget.y -= angDelta;
		break;
	case 'y':
		if (ctrlMode == 0) joints[6].targetAngle += 0.05;
		else angTarget.z += angDelta;
		break;
	case 'h':
		if (ctrlMode == 0) joints[6].targetAngle -= 0.05;
		else angTarget.z -= angDelta;
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
	/*for (auto &jnt : joints)
	{
		jnt.boundJointTargetAngle();
	}*/
	//std::cout << joints[3].targetAngle << std::endl;
}

static void simLoop(int pause)
{
	dSpaceCollide(space, 0, &nearCallback);
	command(' ');

	if (ct%500==0)
		std::cout << "Ctrl Mode= " << ctrlMode 
		<< ", Time= "<< ct*tStep<<"\n";
	if (ctrlMode == 1)
		inverseKinematicsGlobal();
	else if (ctrlMode == 2)
		inverseKinematicsEE();

	jointControl();
	gripperControl();	

	//-----------------------

	dWorldStep(world, tStep);

	// collision setup
	dJointGroupEmpty(contactGroup);


	// draw arms and gripper
	drawArms();

	// draw objects
	drawObjects();
	ct++;
}

void inverseKinematicsGlobal()
{
	// get current joint angles
	std::vector<double> jntAngs;
	getJointAngles(jntAngs);

	//----- forward kinematics
	// update DH parameters
	Eigen::Matrix4d tm_0n, tmat;
	for (int i = 0; i < 6; ++i)
	{
		dhVec[i].setTheta(jntAngs[i]);
		dhVec[i].updateTM();
	}

	// Jacobain matrix
	Eigen::MatrixXd Jg, J_star;
	JgDHs(dhVec, tm_0n, Jg);
	// Jacobian DLS Inverse	
	double k = 0.1;
	JgDLSInverse(Jg, k, J_star);

	// ===== define desired trajectory =====
	Point3 pnow, pd, pd_d, wd, wd_d;
	
	// tip speed
	/*double speedT = 0.3;
	double speedR = 0.3;
	pd_d.x = (2.0*int(posTarget.x >= 0) - 1.0)*int(posTarget.x > 0)*speedT;
	pd_d.y = (2.0*int(posTarget.y >= 0) - 1.0)*int(posTarget.y > 0)*speedT;
	pd_d.z = (2.0*int(posTarget.z >= 0) - 1.0)*int(posTarget.z > 0)*speedT;
	wd.x= (2.0*int(angTarget.x >= 0) - 1.0)*int(angTarget.x > 0)*speedR;
	wd.y = (2.0*int(angTarget.y >= 0) - 1.0)*int(angTarget.x > 0)*speedR;
	wd.z = (2.0*int(angTarget.z >= 0) - 1.0)*int(angTarget.x > 0)*speedR;*/

	Point3 ep, eo;
	// tip position
	pnow.x = tm_0n(0, 3); pnow.y = tm_0n(1, 3); pnow.z = tm_0n(2, 3);
	// position error
	ep = posTarget;

	// orientation error
	Quaternion quatd, quat;
	Eigen::Matrix3d rmd, rmI;
	rmI = Eigen::Matrix3d::Identity();
	rot_zyx(angTarget.z, angTarget.y, angTarget.x, rmd);
	quaternionFromRM(rmI, quat);
	quaternionFromRM(rmd, quatd);
	eo = quatd - quat;

	// --- estimate joint speeds
	double kp = 50;
	double ko = 50;
	Eigen::VectorXd ev(6), qdot(6);
	ev << pd_d.x + kp*ep.x, pd_d.y + kp*ep.y, pd_d.z + kp*ep.z,
		wd_d.x + ko*eo.x, wd_d.y + ko*eo.y, wd_d.z + ko*eo.z;
	qdot = J_star*ev;
	
	// ----- Euler integration
	for (int i = 0; i < 6; ++i)
	{
		jntAngs[i] += qdot(i)*tStep;
		// update target angles
		joints[i + 1].targetAngle = jntAngs[i];
	}

	if (ct % 1000 == 0)
	{
		std::cout << "Joint target angle: \n";
		for (int i = 0; i < 6; ++i)
		{
			std::cout << joints[i + 1].targetAngle << ",";
		}
		std::cout << "\n";
		std::cout << "qdot: ";
		for (int i = 0; i < 6; ++i)
		{
			std::cout << qdot(i) << ",";
		}
		std::cout << "\neo= " << eo << "\n";
		std::cout << "quatd= " << quatd << "\n";

		std::cout << "posTarget= " << posTarget;
		std::cout << "angTarget= " << angTarget;
		//std::cout << "angle= " << dhVec[1].getTheta() << " ODE angle= " << dJointGetHingeAngle(joints[2].jid) << "\n";
		std::cout << "tip location= " << tm_0n(0, 3) << ", " << tm_0n(1, 3) << ", " << tm_0n(2, 3) << "\n";
		const dReal *pos = dBodyGetPosition(gripperParts[0].bid);
		std::cout << "ODE tip location= " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		//std::cout << " ODE tip location= " << pos[2] << "\n";
	}

}

void inverseKinematicsEE()
{
	// get current joint angles from ODE
	std::vector<double> jntAngs;
	getJointAngles(jntAngs);

	//----- forward kinematics
	// update DH parameters
	Eigen::Matrix4d tm_0n, tmat;
	for (int i = 0; i < 6; ++i)
	{
		dhVec[i].setTheta(jntAngs[i]);
		dhVec[i].updateTM();
	}

	// Jacobain matrix
	Eigen::MatrixXd Jg, J_star;
	JgDHs(dhVec, tm_0n, Jg);
	// convert Jacobian to frame n
	Eigen::MatrixXd rn(6, 6);
	Eigen::Matrix3d rm_0n;
	rm_0n = tm_0n.block(0, 0, 3, 3);
	rn = Eigen::MatrixXd::Zero(6, 6);
	rn.block(0, 0, 3, 3) = rm_0n.transpose();
	rn.block(3, 3, 3, 3) = rm_0n.transpose();
	Jg = rn*Jg;


	// Jacobian DLS Inverse	
	double k = 0.2; //0.2;
	double JMag = std::abs(Jg.determinant());

	if (JMag > 0.03)
	{
		J_star = Jg.inverse();
	}
	else
	{
		JgDLSInverse(Jg, k, J_star);
	}
		

	// ===== define desired trajectory =====
	Point3 pnow, pd, pd_d, wd, wd_d;
	Point3 ep, eo;

	// tip position
	pnow.x = tm_0n(0, 3); pnow.y = tm_0n(1, 3); pnow.z = tm_0n(2, 3);
	// position error in n frame
	ep.x = posTarget.x; ep.y = posTarget.y; ep.z = posTarget.z;

	// orientation error
	Quaternion quatd, quat;
	Eigen::Matrix3d rmd, rmI;
	rmI = Eigen::Matrix3d::Identity();
	rot_zyx(angTarget.z, angTarget.y, angTarget.x, rmd);
	
	quaternionFromRM(rmI, quat);
	quaternionFromRM(rmd, quatd);  // convert to local frame
	eo = quatd- quat;

	// --- estimate joint speeds
	double kp = 50;
	double ko = 50;
	Eigen::VectorXd ev(6), qdot(6);
	ev << pd_d.x + kp*ep.x, pd_d.y + kp*ep.y, pd_d.z + kp*ep.z,
		wd_d.x + ko*eo.x, wd_d.y + ko*eo.y, wd_d.z + ko*eo.z;
	qdot = J_star*ev;

	// ----- Euler integration
	for (int i = 0; i < 6; ++i)
	{
		jntAngs[i] += qdot(i)*tStep;
		// update target angles
		joints[i + 1].targetAngle = jntAngs[i];
	}

	if (ct % 1000 == 0)
	{
		std::cout << "Joint target angle: \n";
		for (int i = 0; i < 6; ++i)
		{
			std::cout << joints[i + 1].targetAngle << ",";
		}
		std::cout << "\n";
		std::cout << "qdot: ";
		for (int i = 0; i < 6; ++i)
		{
			std::cout << qdot(i) << ",";
		}
		std::cout << "\neo= " << eo << "\n";
		std::cout << "quatd= " << quatd << "\n";

		std::cout << "posTarget= " << posTarget;
		std::cout << "angTarget= " << angTarget;
		//std::cout << "angle= " << dhVec[1].getTheta() << " ODE angle= " << dJointGetHingeAngle(joints[2].jid) << "\n";
		std::cout << "tip location= " << tm_0n(0, 3) << ", " << tm_0n(1, 3) << ", " << tm_0n(2, 3) << "\n";
		std::cout << "Jg det= " << JMag << "\n";
		//const dReal *pos = dBodyGetPosition(gripperParts[0].bid);
		//std::cout << " ODE tip location= " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		//std::cout << " ODE tip location= " << pos[2] << "\n";
	}
	/*
	if (ct % 200 == 0)
	{
	std::cout << "J_star: \n" << J_star<<"\n";
	}*/

}


void inverseKinematicsAuto()
{
	if (ct >= 500) return;
	// define desired trajectory
	Point3 pnow, pd, pd_d, wd, wd_d;

	pd.x = -0.001*ct;
	pd.y = 0.001*ct;
	pd.z = 1.3 - 0.001*ct;
	pd_d.x = 0.1;
	pd_d.y = 0.1;
	pd_d.z = 0.1;
	wd.y = 0.1;
	wd_d.y = 0.1;
		
	// get current joint angles
	std::vector<double> jntAngs;
	getJointAngles(jntAngs);
	
	//----- forward kinematics
	// update DH parameters
	Eigen::Matrix4d tm_0n, tmat;
	for (int i = 0; i < 6; ++i)
	{
		dhVec[i].setTheta(jntAngs[i]);
		dhVec[i].updateTM();
	}
	
	// Jacobain matrix
	Eigen::MatrixXd Jg, J_star;
	JgDHs(dhVec, tm_0n, Jg);
	// Jacobian DLS Inverse	
	double k = 0.1;
	JgDLSInverse(Jg, k, J_star);
	
	

	// --- calculate Errors
	Quaternion quatd, quat;
	Point3 ep, eo;
	// position error
	pnow.x = tm_0n(0, 3); pnow.y = tm_0n(1, 3); pnow.z = tm_0n(2, 3);
	ep = pd - pnow;
	// orientation error
	Eigen::Matrix3d rmd, rm;
	rm = tm_0n.block(0, 0, 3, 3);
	roty(45. * M_PI / 180.0, rmd);
	quaternionFromRM(rm, quat);
	quaternionFromRM(rmd, quatd);
	eo = quatd - quat;

	// --- estimate joint speeds
	double kp = 50;
	double ko = 50;
	Eigen::VectorXd ev(6), qdot(6);
	ev << pd_d.x + kp*ep.x, pd_d.y + kp*ep.y, pd_d.z + kp*ep.z,
		wd_d.x + ko*eo.x, wd_d.y + ko*eo.y, wd_d.z + ko*eo.z;
	qdot = J_star*ev;
	

	// ----- Euler integration
	for (int i = 0; i < 6; ++i)
	{
		jntAngs[i] += qdot(i)*tStep;
		// update target angles
		joints[i + 1].targetAngle = jntAngs[i];
	}

	

	if (ct % 20 == 0)
	{
		std::cout << "Joint target angle: \n";
		for (int i = 0; i < 6; ++i)
		{
			std::cout << joints[i + 1].targetAngle << ",";
		}
		std::cout << "\n";
		std::cout << "qdot: ";
		for (int i = 0; i < 6; ++i)
		{
			std::cout << qdot(i) << ",";
		}
		std::cout << "\neo= " << eo << "\n";
		std::cout << "quatd= " << quatd << "\n";
		
		//std::cout << "angle= " << dhVec[1].getTheta() << " ODE angle= " << dJointGetHingeAngle(joints[2].jid) << "\n";
		std::cout << "tip location= " << tm_0n(0, 3) << ", " << tm_0n(1, 3) << ", " << tm_0n(2, 3) << "\n";
		//const dReal *pos = dBodyGetPosition(gripperParts[0].bid);
		//std::cout << " ODE tip location= " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\n";
		//std::cout << " ODE tip location= " << pos[2] << "\n";
	}
	/*
	if (ct % 200 == 0)
	{
		std::cout << "J_star: \n" << J_star<<"\n";
	}*/

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
