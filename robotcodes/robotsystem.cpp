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
	rmat << 1,0,0,
		0, cth, -sth,
		0, sth, cth;
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

void rot_zyx(double thz, double thy, double thx, Eigen::Matrix3d & rmat)
{
	rmat << cos(thy)*cos(thz), sin(thx)*sin(thy)*cos(thz) - sin(thz)*cos(thx), sin(thx)*sin(thz) + sin(thy)*cos(thx)*cos(thz),
		sin(thz)*cos(thy), sin(thx)*sin(thy)*sin(thz) + cos(thx)*cos(thz), -sin(thx)*cos(thz) + sin(thy)*sin(thz)*cos(thx),
		-sin(thy), sin(thx)*cos(thy), cos(thx)*cos(thy);
}

/* obtain 4x4 transformation matrix out of DM parameters*/
void tmDH(const DHParameter & dh, Eigen::Matrix4d & tm)
{
	double a = dh.getA();
	double d = dh.getD();
	double cth = std::cos(dh.getTheta());
	double sth = std::sin(dh.getTheta());
	double ca = std::cos(dh.getAlpha());
	double sa = std::sin(dh.getAlpha());

	tm << cth, -sth*ca, sth*sa, a*cth,
		sth, cth*ca, -cth*sa, a*sth,
		0, sa, ca, d,
		0, 0, 0, 1;
}

void tmDHs(const std::vector<DHParameter>& dhs, Eigen::Matrix4d & tm)
{
	tm = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d tmat;
	for (auto dh : dhs)
	{
		tmDH(dh, tmat);
		tm = tm*tmat;
	}
}

void tmDHs(const std::vector<DHParameter>& dhs, int iFirst, int iLast, Eigen::Matrix4d & tm)
{
	// initialize tm
	tm = Eigen::Matrix4d::Identity();

	Eigen::Matrix4d tmat;
	int npt = int(dhs.size());

	// error protection
	if (iLast < iFirst)
		throw std::runtime_error("Last index is smaller than first");


	for (int i = iFirst; i <= std::min(npt - 1, iLast); ++i)
	{
		tmDH(dhs[i], tmat);
		tm = tm*tmat;
	}
}

/* obtain geometric Jacobian from Denavit-Hartenburg parameters */
void JgDHs(const std::vector<DHParameter>& dhs, Eigen::Matrix4d &tm_0n, 
	Eigen::MatrixXd & Jg)
{
	int ncol = int(dhs.size());
	int nrow = 6;  // 6 dof
	Jg.resize(nrow, ncol);

	// forward dynamics
	Eigen::Matrix4d tm;
	tm_0n = Eigen::Matrix4d::Identity();
	for (int j = 0; j < ncol; ++j)
	{
		dhs[j].getTM(tm);
		tm_0n = tm_0n*tm;		
	}
	// build jacobian
	Eigen::Vector3d p, pj, zj, jp;
	Eigen::Matrix4d tmj;
	p<< tm_0n(0,3), tm_0n(1,3), tm_0n(2,3);
	tmj = Eigen::Matrix4d::Identity();

	for (int j = 0; j < ncol; ++j)
	{
		dhs[j].getTM(tm);
		pj << tmj(0, 3), tmj(1, 3), tmj(2, 3);
		zj << tmj(0, 2), tmj(1, 2), tmj(2, 2);
		if (dhs[j].jtype == JointType::Prismatic) // prismatic joint
		{
			// position part
			Jg(0, j) = zj(0); Jg(1, j) = zj(1); Jg(2, j) = zj(2);
			// orientation part
			Jg(3, j) = 0.0; Jg(4, j) = 0.0; Jg(5, j) = 0.0;			
		}
		else  // revolute joint
		{
			jp = zj.cross(p - pj);
			// position part
			Jg(0, j) = jp(0); Jg(1, j) = jp(1); Jg(2, j) = jp(2);
			// orientation part
			Jg(3, j) = zj(0); Jg(4, j) = zj(1); Jg(5, j) = zj(2);
		}
		tmj = tmj*tm;
	}
}

/* Jacobain Right Pseudo inverse*/
void JgRPInverse(const Eigen::MatrixXd & Jg, Eigen::MatrixXd & J_plus)
{
	Eigen::MatrixXd tmp;
	tmp = Jg*(Jg.transpose());
	J_plus = Jg.transpose()*(tmp.inverse());
}

/* Jacobian Damped Least Square inverse*/
void JgDLSInverse(const Eigen::MatrixXd & Jg, double k, Eigen::MatrixXd & J_star)
{
	Eigen::MatrixXd tmp;
	int nrow = Jg.rows();
	tmp= (Jg*Jg.transpose() + (k*k)*Eigen::MatrixXd::Identity(nrow,nrow));
	J_star = Jg.transpose()*(tmp.inverse());
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
		tmDH(dm, tmat1);
		tmat = tmat*tmat1;
		// cm position
		Point3 end(tmat(0, 3), tmat(1, 3), tmat(2, 3));
		cm = 0.5*(end + end_prev);
		end_prev = end;  // update
		// joint axis (z axis)
		axis.x = tmat(0, 2); axis.y = tmat(1, 2); axis.z = tmat(2, 2);
		// link length
		double lnkL = std::sqrt(dm.getA()*dm.getA() + dm.getD()*dm.getD());

		// add to output
		jntAxes.push_back(axis);
		jntAnchors.push_back(end);
		linkCMs.push_back(cm);
		linkLengths.push_back(lnkL);
	}
}

void quaternionFromRM(const Eigen::Matrix3d & rm, Quaternion & quat)
{
	quat.e0 = 0.5*std::sqrt(rm.trace()+1.0);
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;
	r11= rm(0, 0); r12 = rm(0, 1); r13 = rm(0, 2);
	r21 = rm(1, 0); r22 = rm(1, 1); r23 = rm(1, 2);
	r31 = rm(2, 0); r32 = rm(2, 1); r33 = rm(2, 2);

	double sgn;
	sgn= (int((r32 - r23)>0)*2.0)-1.0;
	quat.e1 = 0.5*sgn*std::sqrt(r11 - r22 - r33 + 1.0);
	sgn = (int((r13 - r31)>0)*2.0) - 1.0;
	quat.e2 = 0.5*sgn*std::sqrt(r22 - r33 - r11 + 1.0);
	sgn = (int((r21 - r12)>0)*2.0) - 1.0;
	quat.e3 = 0.5*sgn*std::sqrt(r33 - r11 - r22 + 1.0);
}


Point3 operator-(const Quaternion & q1, const Quaternion & q2)
{
	Point3 out;
	Eigen::Vector3d ev1, ev2, eo;
	ev1<< q1.e1,q1.e2,q1.e3;
	ev2<< q2.e1,q2.e2,q2.e3;
	eo = q2.e0*ev1 - q1.e0*ev2 - ev1.cross(ev2);
	out.x = eo(0);  out.y = eo(1); out.z = eo(2);
	return out;
}

std::ostream & operator<<(std::ostream & output, const Quaternion & q)
{
	output << q.e0 << ", " << q.e1 << ", " << q.e2 << ", "<< q.e3<< "\n";
	return output;
}

void TransMat::getEigenMat(Eigen::Matrix4d &tm)
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			tm(i, j) = elem[i][j];
		}
	}
}

void DHParameter::updateTM()
{
	double cth = std::cos(theta);
	double sth = std::sin(theta);
	double ca = std::cos(alpha);
	double sa = std::sin(alpha);

	tm.elem[0][0] = cth;
	tm.elem[0][1] = -sth*ca;
	tm.elem[0][2] = sth*sa;
	tm.elem[0][3] = a*cth;
	tm.elem[1][0] = sth;
	tm.elem[1][1] = cth*ca;
	tm.elem[1][2] = -cth*sa;
	tm.elem[1][3] = a*sth;
	tm.elem[2][0] = 0;
	tm.elem[2][1] = sa;
	tm.elem[2][2] = ca;
	tm.elem[2][3] = d;
	tm.elem[3][0] = 0.0;
	tm.elem[3][1] = 0.0;
	tm.elem[3][2] = 0.0;
	tm.elem[3][3] = 1.0;
}

void DHParameter::getTM(Eigen::Matrix4d & tm) const
{
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			tm(i, j) = this->tm.elem[i][j];
		}
	}
}
