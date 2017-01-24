#include "ConvexAlg.h"

#include <iostream>
#include <fstream>
using namespace BM;

/* return false is not colinear*/
bool CH::colinear3PtTest(const Point2 & pt1, const Point2 & pt2, const Point2 & pt3)
{
	// calculate vector p1 ->p2
	Point2 vec12;
	vec12 = pt2 - pt1;
	// calculate vector p1->p3
	Point2 vec13;
	vec13 = pt3 - pt1;
	// check vector length
	double mag12 = vec12.mag();
	double mag13 = vec13.mag();
	if (mag12 == 0 || mag13 == 0)
	{
		return false;
	}

	// calculate normalized dot product
	double dotProd = vec12.dot(vec13)/mag12/mag13;

	if (std::abs(dotProd) == 1.0)  // colinear
	{
		return true;
	}
	else
	{
		return false;
	}
}

std::vector<Polar> CH::getConvexPolars(const std::vector<Polar>& polarList)
{
	std::vector<Polar> chPolars;
	int npt = int(polarList.size());

	int iP = 2;  // index for polars1 vector

	chPolars.push_back(polarList[0]);
	chPolars.push_back(polarList[1]);
	chPolars.push_back(polarList[2]);

	while (true)
	{
		Polar pr1, pr2, pr3;

		pr1 = *(chPolars.end() - 3);
		pr2 = *(chPolars.end() - 2);
		pr3 = *(chPolars.end() - 1);

		// get vectors
		Point2 v_21, v_2p, v_23;
		v_21.x = pr1.r*std::cos(pr1.theta) - pr2.r*std::cos(pr2.theta);
		v_21.y = pr1.r*std::sin(pr1.theta) - pr2.r*std::sin(pr2.theta);
		v_23.x = pr3.r*std::cos(pr3.theta) - pr2.r*std::cos(pr2.theta);
		v_23.y = pr3.r*std::sin(pr3.theta) - pr2.r*std::sin(pr2.theta);
		v_2p.x = -pr2.r*std::cos(pr2.theta);
		v_2p.y = -pr2.r*std::sin(pr2.theta);

		// calculate angles
		double alpha = std::acos(v_21.dot(v_2p) / v_21.mag() / v_2p.mag());
		double beta = std::acos(v_2p.dot(v_23) / v_2p.mag() / v_23.mag());

		if ((alpha + beta) >= M_PI)  // concave
		{
			// remove mid point
			chPolars.erase(chPolars.end() - 2);
		}
		else  // convex, shift iterator forwards
		{
			iP++;
			// termination condition
			if (iP >= npt)
			{
				break;
			}
			chPolars.push_back(polarList[iP]);
		}
	}
	return chPolars;
}

std::vector<Point2> CH::grahamConvexHull(std::vector<Point2> points)
{
	std::vector<Point2> ch;
	int npt = static_cast<int>(points.size());
	// ====== Step 1: obtain origin p =====
	Point2 centroid;
	for (int i = 0; i < npt - 2; ++i)
	{
		if (!colinear3PtTest(points[i], points[i + 1], points[i + 2]))
		{
			// calculate centroid
			centroid = (points[i] + points[i + 1] + points[i + 2]) / 3.0;
			break;
		}
	}
	
	// ===== Step 2: convert to polar coord ========
	std::vector<Polar> polars;
	double r, th;
	for (auto pt : points)
	{
		double dx= pt.x - centroid.x;
		double dy = pt.y - centroid.y;
		Polar tmp;
		r = std::sqrt(dx*dx + dy*dy);
		th = std::atan2(dy, dx);  // [-pi,pi]
		// correct theta to [0, 2*pi]
		th = (th<0.0) ? (2*M_PI+th): th;
		// add to vector
		tmp.r = r;
		tmp.theta = th;
		polars.push_back(tmp);
	}
	// ===== Step 3: sort by angle =====
	std::sort(polars.begin(), polars.end(),
		[](const Polar &p1, const Polar &p2) {return p1.theta < p2.theta; });
	
	// ==== Step 4: eliminate points with smaller amp =====
	std::vector<Polar> polars1;
	for (int i = 0; i< int(polars.size()) - 1; ++i)
	{
		if (polars[i].theta == polars[i + 1].theta)
		{
			Polar tmp = (polars[i].r < polars[i + 1].r) ? polars[i] : polars[i + 1];
			polars1.push_back(tmp);
		}
		else
		{
			polars1.push_back(polars[i]);
			if (i == (int(polars.size())-2)) polars1.push_back(polars[i + 1]);
		}
	}	

	// ==== Step 5: collect points
	std::vector<Polar> chPolars;
	chPolars = getConvexPolars(polars1);

	// close the loop
	std::rotate(chPolars.begin(), chPolars.end() - 3, chPolars.end());
	std::vector<Polar> chPolarsOut;
	chPolarsOut = getConvexPolars(chPolars);

	// convert Polar vector to vector of point2
	for (auto pr : chPolarsOut)
	{
		Point2 pt;
		pt.x = pr.r*std::cos(pr.theta) + centroid.x;
		pt.y = pr.r*std::sin(pr.theta) + centroid.y;
		ch.push_back(pt);	
	}
	return ch;
}
